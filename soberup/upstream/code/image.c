/**************************************************************
 * 文件名:  image.c
 * 创建时间: 2026年1月30日 下午8:26:19
 * 说明: 摄像头图像处理 —— 局部自适应阈值二值化
 *
 * 算法概览 (CPU1 每帧调用):
 *   1. otsuThreshold_fast(): 先把图像按 6列×4行 网格分块, 稀疏采样
 *      (x步长4, y步长2) 建 32-bin 直方图; 中间4列汇总为全局直方图,
 *      用全局 OTSU 求基准阈值; 每块再用局部 25% 分位 / 最暗峰与全局
 *      参考作差得到补偿 raise (夹到 [0,30]), 异常亮块用近邻中位数修复。
 *   2. binarization(): 逐像素阈值 = 全局基准 + 纵向插值后的 raise,
 *      用 lt.bu 指令一次比较 4 像素写出二值图。
 *
 * 内存说明: 大数组与帧间状态放 CPU1 DSRAM; img_avg_gray 供 CPU0 菜单显示。
 **************************************************************/
#include "image.h"
#include "Cpu/Std/IfxCpu_Intrinsics.h"

/* ========================= 网格与直方图参数 ========================= */
#define LOCAL_GRID_COLS        6    /* 横向分块数 */
#define LOCAL_GRID_ROWS        4    /* 纵向分块数 */
#define LOCAL_HIST_BINS        32   /* 直方图桶数 (灰度 >> SHIFT 得桶号) */
#define LOCAL_HIST_SHIFT       3    /* 每桶覆盖 2^3 = 8 级灰度 */
#define LOCAL_SAMPLE_STEP_X    4    /* 采样横向步长 */
#define LOCAL_SAMPLE_STEP_Y    2    /* 采样纵向步长 */

/* ========================= 局部亮度补偿参数 ========================= */
#define LOCAL_PERCENTILE_NUM   1    /* 分位数分子 */
#define LOCAL_PERCENTILE_DEN   4    /* 分位数分母 (1/4 = 25% 分位) */
#define LOCAL_MAX_RAISE        30   /* 单块补偿上限; 超过视为异常亮块 */
#define LOCAL_PEAK_SUPPORT_DEN 8U   /* 直方图峰的最小支撑 = count/8 */
#define LOCAL_INVALID_VALUE    255U /* 无效值标记 */

/* ========================= 全局 OTSU 参数 ========================= */
#define GLOBAL_TILE_COL_FIRST  1    /* 参与全局统计的起始列 */
#define GLOBAL_TILE_COL_LAST   (LOCAL_GRID_COLS - 2) /* 参与全局统计的结束列 */
#define OTSU_MIN_CLASS_GAP     30.0f  /* OTSU 可信所需最小类间差 */
#define OTSU_MIN_THRESHOLD     40   /* 阈值下限 */
#define OTSU_MAX_THRESHOLD     180  /* 阈值上限 */

/* ========================= 纵向插值参数 ========================= */
#define INTERP_SCALE           256U /* 定点插值比例 (1.0 = 256) */

/* ========================= CPU1 DSRAM 状态 =========================
 * 大数组与帧间状态放在 CPU1 DSRAM, 避免超过 CPU1 用户栈 (2KB) 限制。
 */
#pragma section all "cpu1_dsram"
IFX_ALIGN(4) uint8 mt9v03x_image_bin[MT9V03X_H][MT9V03X_W];

/* 直方图: 单块局部直方图 + 中间列汇总的全局直方图 */
static uint16 local_histogram[LOCAL_HIST_BINS];
static uint16 global_histogram[LOCAL_HIST_BINS];

/* 每块状态: 亮度补偿值 / 最暗峰 / 是否有效 / 上帧补偿(供无效时回填) */
static uint8 local_raise[LOCAL_GRID_ROWS][LOCAL_GRID_COLS];
static uint8 local_dark_peak[LOCAL_GRID_ROWS][LOCAL_GRID_COLS];
static uint8 local_valid[LOCAL_GRID_ROWS][LOCAL_GRID_COLS];
static uint8 previous_local_raise[LOCAL_GRID_ROWS][LOCAL_GRID_COLS];

/* 预计算的坐标映射: 列边界 / 每行的上块行号 / 行间插值权重 */
static uint8 x_tile_boundary[LOCAL_GRID_COLS + 1];
static uint8 y_top_node[MT9V03X_H];
static uint8 y_weight[MT9V03X_H];

/* 一次性初始化与帧间平滑标志 */
static uint8 interpolation_ready;   /* 坐标映射表是否已建立 */
static uint8 local_history_ready;   /* 是否已有上帧 raise 可回填 */
static uint8 previous_threshold = 100U; /* 上帧全局阈值 (EMA 平滑) */
#pragma section all restore

/* 菜单显示用: 全局采样区平均灰度 (CPU1 每帧更新, CPU0 读取显示) */
uint8 img_avg_gray = 0;

/* ========================= 像素安全访问 ========================= */

uint8 safe_access_binimg(int x, int y)
{
    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
    {
        return mt9v03x_image_bin[y][x];
    }
    return 0U;
}

uint8 safe_access_img(int x, int y)
{
    if (x >= 0 && x < MT9V03X_W - 1 && y >= 0 && y < MT9V03X_H)
    {
        return mt9v03x_image[y][x];
    }
    return 0U;
}

/* ========================= 直方图工具 ========================= */

/* 返回直方图的 NUM/DEN 分位对应灰度 (桶中心值) */
static uint8 histogram_percentile_32(
    const uint16 *histogram,
    uint32 count)
{
    uint32 target;
    uint32 cumulative = 0U;
    int bin;

    if (count == 0U)
    {
        return 0U;
    }

    target = (count * LOCAL_PERCENTILE_NUM
            + LOCAL_PERCENTILE_DEN - 1U) / LOCAL_PERCENTILE_DEN;
    if (target == 0U)
    {
        target = 1U;
    }

    for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
    {
        cumulative += histogram[bin];
        if (cumulative >= target)
        {
            return (uint8)((bin << LOCAL_HIST_SHIFT)
                         + (1U << (LOCAL_HIST_SHIFT - 1)));
        }
    }

    return 255U;
}

/* 桶 bin 及左右邻居的计数和 (越界邻居按 0 计) */
static uint32 histogram_support_32(const uint16 *histogram, int bin)
{
    uint32 support = histogram[bin];

    if (bin > 0)
    {
        support += histogram[bin - 1];
    }
    if (bin + 1 < LOCAL_HIST_BINS)
    {
        support += histogram[bin + 1];
    }
    return support;
}

/* 加权平滑: 自身×2 + 左右邻居 (越界按 0 计) */
static uint32 histogram_smooth_32(const uint16 *histogram, int bin)
{
    uint32 value;

    if (bin < 0 || bin >= LOCAL_HIST_BINS)
    {
        return 0U;
    }

    value = (uint32)histogram[bin] * 2U;
    if (bin > 0)
    {
        value += histogram[bin - 1];
    }
    if (bin + 1 < LOCAL_HIST_BINS)
    {
        value += histogram[bin + 1];
    }
    return value;
}

/* 找到第一个"平滑后局部最大且支撑 ≥ count/8"的峰, 返回其中心灰度 */
static uint8 find_first_supported_peak_32(
    const uint16 *histogram,
    uint32 count)
{
    uint32 minimum_support;
    int bin;

    if (count == 0U)
    {
        return LOCAL_INVALID_VALUE;
    }

    minimum_support = (count + LOCAL_PEAK_SUPPORT_DEN - 1U)
                    / LOCAL_PEAK_SUPPORT_DEN;
    for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
    {
        uint32 current = histogram_smooth_32(histogram, bin);

        if (current >= histogram_smooth_32(histogram, bin - 1)
            && current >= histogram_smooth_32(histogram, bin + 1)
            && histogram_support_32(histogram, bin) >= minimum_support)
        {
            return (uint8)((bin << LOCAL_HIST_SHIFT)
                         + (1U << (LOCAL_HIST_SHIFT - 1)));
        }
    }

    return LOCAL_INVALID_VALUE;
}

/* 在不超过 maximum_gray 的灰度范围内找全局最强峰, 输出其中心灰度 */
static uint8 find_global_peak_32(uint8 maximum_gray, uint8 *peak)
{
    uint32 best_smooth = 0U;
    uint8 found = 0U;
    int maximum_bin = maximum_gray >> LOCAL_HIST_SHIFT;
    int bin;

    for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
    {
        uint8 center = (uint8)((bin << LOCAL_HIST_SHIFT)
                             + (1U << (LOCAL_HIST_SHIFT - 1)));
        uint32 smooth;

        if (bin > maximum_bin)
        {
            break;
        }
        if (histogram_support_32(global_histogram, bin) == 0U)
        {
            continue;
        }

        smooth = histogram_smooth_32(global_histogram, bin);
        if (found == 0U || smooth > best_smooth)
        {
            found = 1U;
            best_smooth = smooth;
            *peak = center;
        }
    }

    return found;
}

/* 就地插入排序后取中位数; 偶数个取中间两数平均 */
static uint8 median_u8(uint8 *values, int count)
{
    int i;

    for (i = 1; i < count; i++)
    {
        uint8 value = values[i];
        int j = i;

        while (j > 0 && values[j - 1] > value)
        {
            values[j] = values[j - 1];
            j--;
        }
        values[j] = value;
    }

    if ((count & 1) != 0)
    {
        return values[count / 2];
    }
    return (uint8)(((uint16)values[count / 2 - 1]
                  + values[count / 2]) / 2U);
}

/* ========================= 局部补偿修复 ========================= */

/* 全块无效时用上帧 raise 回填; 无历史则填 0 */
static void restore_local_raise_history(void)
{
    int y;
    int x;

    for (y = 0; y < LOCAL_GRID_ROWS; y++)
    {
        for (x = 0; x < LOCAL_GRID_COLS; x++)
        {
            local_raise[y][x] = (local_history_ready != 0U)
                ? previous_local_raise[y][x] : 0U;
        }
    }
}

/* 异常亮块: 用曼哈顿距离最近的有效块的 raise 中位数填充 */
static void repair_invalid_local_raise(void)
{
    uint8 valid_count = 0U;
    int y;
    int x;

    for (y = 0; y < LOCAL_GRID_ROWS; y++)
    {
        for (x = 0; x < LOCAL_GRID_COLS; x++)
        {
            if (local_valid[y][x] != 0U)
            {
                valid_count++;
            }
        }
    }

    if (valid_count == 0U)
    {
        restore_local_raise_history();
        return;
    }

    for (y = 0; y < LOCAL_GRID_ROWS; y++)
    {
        for (x = 0; x < LOCAL_GRID_COLS; x++)
        {
            uint8 nearest_values[LOCAL_GRID_ROWS * LOCAL_GRID_COLS];
            int nearest_count = 0;
            int minimum_distance = LOCAL_GRID_ROWS + LOCAL_GRID_COLS;
            int source_y;
            int source_x;

            if (local_valid[y][x] != 0U)
            {
                continue;
            }

            for (source_y = 0; source_y < LOCAL_GRID_ROWS; source_y++)
            {
                for (source_x = 0; source_x < LOCAL_GRID_COLS; source_x++)
                {
                    int dy;
                    int dx;
                    int distance;

                    if (local_valid[source_y][source_x] == 0U)
                    {
                        continue;
                    }

                    dy = source_y - y;
                    dx = source_x - x;
                    if (dy < 0)
                    {
                        dy = -dy;
                    }
                    if (dx < 0)
                    {
                        dx = -dx;
                    }
                    distance = dy + dx;

                    if (distance < minimum_distance)
                    {
                        minimum_distance = distance;
                        nearest_count = 0;
                    }
                    if (distance == minimum_distance)
                    {
                        nearest_values[nearest_count++] =
                            local_raise[source_y][source_x];
                    }
                }
            }

            if (nearest_count > 0)
            {
                local_raise[y][x] =
                    median_u8(nearest_values, nearest_count);
            }
            else
            {
                local_raise[y][x] = 0U;
            }
        }
    }

    for (y = 0; y < LOCAL_GRID_ROWS; y++)
    {
        for (x = 0; x < LOCAL_GRID_COLS; x++)
        {
            previous_local_raise[y][x] = local_raise[y][x];
        }
    }
    local_history_ready = 1U;
}

/* ========================= 坐标映射表 ========================= */

/* 第 node 块在 [0, length) 范围内的中心坐标 */
static int grid_center(int node, int length, int node_count)
{
    int start = node * length / node_count;
    int end = (node + 1) * length / node_count;
    return (start + end - 1) / 2;
}

/* 预计算 x 列边界、每行的上块行号与插值权重 (只执行一次) */
static void initialize_interpolation_maps(void)
{
    int first_center;
    int last_center;
    int coordinate;
    int node;

    if (interpolation_ready != 0U)
    {
        return;
    }

    for (node = 0; node <= LOCAL_GRID_COLS; node++)
    {
        x_tile_boundary[node] =
            (uint8)(node * MT9V03X_W / LOCAL_GRID_COLS);
    }

    first_center = grid_center(0, MT9V03X_H, LOCAL_GRID_ROWS);
    last_center = grid_center(
        LOCAL_GRID_ROWS - 1, MT9V03X_H, LOCAL_GRID_ROWS);

    for (coordinate = 0; coordinate < MT9V03X_H; coordinate++)
    {
        if (coordinate <= first_center)
        {
            y_top_node[coordinate] = 0U;
            y_weight[coordinate] = 0U;
        }
        else if (coordinate >= last_center)
        {
            y_top_node[coordinate] = LOCAL_GRID_ROWS - 1;
            y_weight[coordinate] = 0U;
        }
        else
        {
            for (node = 0; node < LOCAL_GRID_ROWS - 1; node++)
            {
                int top_center = grid_center(
                    node, MT9V03X_H, LOCAL_GRID_ROWS);
                int bottom_center = grid_center(
                    node + 1, MT9V03X_H, LOCAL_GRID_ROWS);

                if (coordinate < bottom_center)
                {
                    uint32 weight = (uint32)(coordinate - top_center)
                                  * INTERP_SCALE
                                  / (uint32)(bottom_center - top_center);
                    if (weight > 255U)
                    {
                        weight = 255U;
                    }
                    y_top_node[coordinate] = (uint8)node;
                    y_weight[coordinate] = (uint8)weight;
                    break;
                }
            }
        }
    }

    interpolation_ready = 1U;
}

/* ========================= 二值化 ========================= */

/*
 * LT.BU 同时比较四个无符号字节; 阈值放在左操作数,
 * 因而每个结果字节在 pixel > threshold 时为 0xFF, 否则为 0x00。
 */
static inline uint32 binarize_u8x4(uint32 pixels4, uint32 threshold4)
{
    uint32 output4;

    __asm("lt.bu %0,%1,%2"
          : "=d"(output4)
          : "d"(threshold4), "d"(pixels4));
    return output4;
}

#pragma optimize 2
void binarization(uint8 thres)
{
    int y;

    initialize_interpolation_maps();

    for (y = 0; y < MT9V03X_H; y++)
    {
        uint8 top = y_top_node[y];
        uint8 wy = y_weight[y];
        uint8 *source_row = mt9v03x_image[y];
        uint8 *binary_row = mt9v03x_image_bin[y];
        int node;

        /* 纵向保持平滑, 横向每块共用阈值以缩短逐像素热路径。 */
        for (node = 0; node < LOCAL_GRID_COLS; node++)
        {
            uint16 raise;
            uint16 threshold;
            uint32 threshold4;
            int x;
            int x_end;

            /* 当前行所在上下块行的 raise 做线性插值 */
            if (top >= LOCAL_GRID_ROWS - 1)
            {
                raise = local_raise[LOCAL_GRID_ROWS - 1][node];
            }
            else
            {
                uint16 upper = local_raise[top][node];
                uint16 lower = local_raise[top + 1][node];
                raise = (uint16)(
                    (upper * (INTERP_SCALE - wy) + lower * wy) >> 8);
            }

            /* 本块阈值 = 全局基准 + 局部补偿 */
            threshold = (uint16)thres + raise;
            threshold4 = threshold * 0x01010101U;
            x = x_tile_boundary[node];
            x_end = x_tile_boundary[node + 1];

            /* 分块起点可能不对齐 4 字节, 先逐像素推进到对齐 */
            while (x < x_end && (x & 3) != 0)
            {
                binary_row[x] =
                    ((uint16)source_row[x] > threshold) ? 255U : 0U;
                x++;
            }

            /* 4 像素并行比较 */
            for (; x + 3 < x_end; x += 4)
            {
                uint32 pixels4 = __ld32(&source_row[x]);
                uint32 output4 = binarize_u8x4(pixels4, threshold4);

                __st32(&binary_row[x], output4);
            }

            /* 尾部逐像素 */
            for (; x < x_end; x++)
            {
                binary_row[x] =
                    ((uint16)source_row[x] > threshold) ? 255U : 0U;
            }
        }
    }
}
#pragma endoptimize

/* ========================= 全局 OTSU + 局部补偿 ========================= */

/**
 * @brief 计算全局 OTSU 阈值, 同时生成 6×4 向上局部亮度补偿。
 * @return 经过可信度判断、限幅和帧间平滑的全局阈值。
 */
#pragma optimize 2
uint8 otsuThreshold_fast(void)
{
    uint32 total_count = 0U;   /* 全局采样点总数 */
    uint32 total_gray = 0U;    /* 全局采样灰度(桶中心)加权和 */
    uint32 global_gray_sum = 0U; /* 全局采样区真实灰度累加, 供 img_avg_gray */
    uint8 first_global_bin = LOCAL_INVALID_VALUE;
    uint8 last_global_bin = 0U;
    uint8 global_background;
    uint8 global_peak = 0U;
    uint8 global_peak_valid = 0U;
    uint8 otsu_reliable = 0U;
    uint8 final_threshold = previous_threshold;
    uint32 background_count = 0U;
    uint32 background_sum = 0U;
    uint32 best_background_count = 0U;
    uint32 best_background_sum = 0U;
    uint8 best_threshold = previous_threshold;
    float max_variance = 0.0f;
    int tile_y;
    int tile_x;
    int bin;

    /* 清空全局直方图 */
    for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
    {
        global_histogram[bin] = 0U;
    }

    /* ---- 逐块采样建直方图 ---- */
    for (tile_y = 0; tile_y < LOCAL_GRID_ROWS; tile_y++)
    {
        int y_start = tile_y * MT9V03X_H / LOCAL_GRID_ROWS;
        int y_end = (tile_y + 1) * MT9V03X_H / LOCAL_GRID_ROWS;

        for (tile_x = 0; tile_x < LOCAL_GRID_COLS; tile_x++)
        {
            int x_start = tile_x * MT9V03X_W / LOCAL_GRID_COLS;
            int x_end = (tile_x + 1) * MT9V03X_W / LOCAL_GRID_COLS;
            uint32 local_count = 0U;
            int y;

            for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
            {
                local_histogram[bin] = 0U;
            }

            for (y = y_start; y < y_end; y += LOCAL_SAMPLE_STEP_Y)
            {
                uint8 *row = mt9v03x_image[y];
                int x;

                for (x = x_start; x < x_end; x += LOCAL_SAMPLE_STEP_X)
                {
                    uint8 value = row[x];

                    local_histogram[value >> LOCAL_HIST_SHIFT]++;
                    local_count++;
                    if (tile_x >= GLOBAL_TILE_COL_FIRST
                        && tile_x <= GLOBAL_TILE_COL_LAST)
                    {
                        global_gray_sum += value;
                    }
                }
            }

            /* 块的 25% 分位(亮度参考) 与最暗峰 */
            if (local_count == 0U)
            {
                local_raise[tile_y][tile_x] = LOCAL_INVALID_VALUE;
                local_dark_peak[tile_y][tile_x] = LOCAL_INVALID_VALUE;
            }
            else
            {
                local_raise[tile_y][tile_x] =
                    histogram_percentile_32(local_histogram, local_count);
                local_dark_peak[tile_y][tile_x] =
                    find_first_supported_peak_32(
                        local_histogram,
                        local_count);
            }

            /* 中间列汇总进全局直方图 */
            if (tile_x >= GLOBAL_TILE_COL_FIRST
                && tile_x <= GLOBAL_TILE_COL_LAST)
            {
                for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
                {
                    global_histogram[bin] = (uint16)(
                        global_histogram[bin] + local_histogram[bin]);
                }
            }
        }
    }

    /* 全局直方图统计: 首末非空桶、总采样数与灰度加权和 */
    for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
    {
        uint16 count = global_histogram[bin];

        if (count != 0U)
        {
            uint8 center = (uint8)((bin << LOCAL_HIST_SHIFT)
                                 + (1U << (LOCAL_HIST_SHIFT - 1)));

            if (first_global_bin == LOCAL_INVALID_VALUE)
            {
                first_global_bin = (uint8)bin;
            }
            last_global_bin = (uint8)bin;
            total_count += count;
            total_gray += (uint32)center * count;
        }
    }

    /* 全图近乎单色或采样为空: 无法可靠分前后景 */
    if (total_count == 0U || first_global_bin == last_global_bin)
    {
        restore_local_raise_history();
        return previous_threshold;
    }

    /* 全局采样区平均灰度 (菜单显示用, 桶中心近似) */
    img_avg_gray = (uint8)(global_gray_sum / total_count);

    /* 全局 25% 分位作为背景参考 */
    global_background = histogram_percentile_32(
        global_histogram,
        total_count);

    /* ---- 全局 OTSU 求基准阈值 ---- */
    for (bin = 0; bin < LOCAL_HIST_BINS; bin++)
    {
        uint8 center = (uint8)((bin << LOCAL_HIST_SHIFT)
                             + (1U << (LOCAL_HIST_SHIFT - 1)));
        uint32 foreground_count;
        uint32 foreground_sum;
        float background_mean;
        float foreground_mean;
        float difference;
        float variance;

        background_count += global_histogram[bin];
        background_sum += (uint32)center * global_histogram[bin];

        if (background_count == 0U || background_count >= total_count)
        {
            continue;
        }

        foreground_count = total_count - background_count;
        foreground_sum = total_gray - background_sum;
        background_mean = (float)background_sum / background_count;
        foreground_mean = (float)foreground_sum / foreground_count;
        difference = foreground_mean - background_mean;
        variance = (float)background_count
                 * (float)foreground_count
                 * difference
                 * difference;

        if (variance > max_variance)
        {
            max_variance = variance;
            best_threshold = (uint8)((bin << LOCAL_HIST_SHIFT) + 7);
            best_background_count = background_count;
            best_background_sum = background_sum;
        }
    }

    /* ---- OTSU 可信度: 类间差足够大才采用新阈值 ---- */
    if (best_background_count > 0U
        && best_background_count < total_count)
    {
        uint32 foreground_count = total_count - best_background_count;
        uint32 foreground_sum = total_gray - best_background_sum;
        float background_mean =
            (float)best_background_sum / best_background_count;
        float foreground_mean =
            (float)foreground_sum / foreground_count;
        float class_gap = foreground_mean - background_mean;

        if (class_gap >= OTSU_MIN_CLASS_GAP)
        {
            otsu_reliable = 1U;
            final_threshold = best_threshold;
            global_peak_valid =
                find_global_peak_32(best_threshold, &global_peak);
        }
    }

    /* ---- 逐块局部补偿 ---- */
    for (tile_y = 0; tile_y < LOCAL_GRID_ROWS; tile_y++)
    {
        for (tile_x = 0; tile_x < LOCAL_GRID_COLS; tile_x++)
        {
            uint8 local_reference;
            uint8 global_reference;
            int difference;

            if (local_raise[tile_y][tile_x] == LOCAL_INVALID_VALUE)
            {
                local_valid[tile_y][tile_x] = 0U;
                local_raise[tile_y][tile_x] = 0U;
                continue;
            }

            /* 优先用暗峰对作参考; 不可用时退回分位数对 */
            if (global_peak_valid != 0U
                && local_dark_peak[tile_y][tile_x]
                    != LOCAL_INVALID_VALUE)
            {
                local_reference = local_dark_peak[tile_y][tile_x];
                global_reference = global_peak;
            }
            else
            {
                local_reference = local_raise[tile_y][tile_x];
                global_reference = global_background;
            }

            /* 亮于全局的块抬高阈值; 异常亮块标记无效交给修复 */
            difference = (int)local_reference - (int)global_reference;
            if (difference > LOCAL_MAX_RAISE)
            {
                local_valid[tile_y][tile_x] = 0U;
                local_raise[tile_y][tile_x] = 0U;
            }
            else
            {
                local_valid[tile_y][tile_x] = 1U;
                local_raise[tile_y][tile_x] =
                    (difference > 0) ? (uint8)difference : 0U;
            }
        }
    }

    repair_invalid_local_raise();

    /* OTSU 不可信则沿用上帧阈值 */
    if (otsu_reliable == 0U)
    {
        return previous_threshold;
    }

    /* 限幅 + 帧间 EMA 平滑 */
    if (final_threshold < OTSU_MIN_THRESHOLD)
    {
        final_threshold = OTSU_MIN_THRESHOLD;
    }
    if (final_threshold > OTSU_MAX_THRESHOLD)
    {
        final_threshold = OTSU_MAX_THRESHOLD;
    }

    final_threshold =
        (uint8)(((uint16)previous_threshold * 3U + final_threshold) / 4U);
    previous_threshold = final_threshold;
    return final_threshold;
}
#pragma endoptimize
