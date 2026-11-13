/**************************************************************
 * 文件名:  element_line.c
 * 创建时间: 2026年2月20日 下午7:41:59
 * 说明:
 **************************************************************/
#include "image.h"
#include "tool.h"
#include "maze.h"
#include "zf_device_ips200.h"
#include "zf_device_ips114.h"
#include "element_line.h"

#define FORK_SIDE_EXIT_MIN_Y 20

int element_line_L=0,element_line_R=0,element_line_U=0,element_line_M=0;

Point L_edge[10],R_edge[10],U_edge[10],B_edge[10],M_edge[10];

Point out_L,out_R;

// 假三叉过滤白点计数（调试显示用）
int fork_white_L = 0, fork_white_R = 0;
// 三叉入口调试: 0=通过, 1=L/R点数为0, 2=缺L-only或R-only,
// 3=几何失败, 4=三层无有效出口, 5=假三叉白点过滤失败,
// 6=左右爬虫的穿出边界组合不符合固有几何关系
int fork_dbg_reason = 0;
int fork_dbg_layer = -1;
int fork_dbg_has_only_L = 0, fork_dbg_has_only_R = 0;
int fork_dbg_hit_left = 0, fork_dbg_hit_right = 0, fork_dbg_hit_top = 0;
int fork_dbg_step_L = 0, fork_dbg_step_R = 0;
int fork_dbg_thresh_L = 0, fork_dbg_thresh_R = 0;
int fork_dbg_y_diff = 0, fork_dbg_x_diff = 0;
int fork_dbg_geom_ok = 0, fork_dbg_white_ok = 1;
int fork_dbg_src_L = 0, fork_dbg_src_R = 0;
int fork_dbg_boundary_L = FORK_EXIT_BOUNDARY_NONE;
int fork_dbg_boundary_R = FORK_EXIT_BOUNDARY_NONE;
int angle_detect_lookahead = 20;

static void reset_fork_debug(void)
{
    fork_dbg_reason = 0;
    fork_dbg_layer = -1;
    fork_dbg_has_only_L = 0;
    fork_dbg_has_only_R = 0;
    fork_dbg_hit_left = 0;
    fork_dbg_hit_right = 0;
    fork_dbg_hit_top = 0;
    fork_dbg_step_L = 0;
    fork_dbg_step_R = 0;
    fork_dbg_thresh_L = 0;
    fork_dbg_thresh_R = 0;
    fork_dbg_y_diff = 0;
    fork_dbg_x_diff = 0;
    fork_dbg_geom_ok = 0;
    fork_dbg_white_ok = 1;
    fork_dbg_src_L = 0;
    fork_dbg_src_R = 0;
    fork_dbg_boundary_L = FORK_EXIT_BOUNDARY_NONE;
    fork_dbg_boundary_R = FORK_EXIT_BOUNDARY_NONE;
}

/*
static int is_legal_fork_exit_pair(int left_boundary, int right_boundary)
{
    return route_classify_fork_exit_pair(left_boundary, right_boundary) !=
        FORK_EXIT_PAIR_INVALID;
}
*/

//-------------------------------------------------------------------------------------------------------------------
// line_W 和 line_H 等函数的位置
//-------------------------------------------------------------------------------------------------------------------
uint8_t line_W(int hight){
    for(int i=30;i<MT9V03X_W-30;i++){
        if(safe_access_binimg(i-2,hight)==0){
            if(safe_access_binimg(i-1,hight)==0){
                if(safe_access_binimg(i,hight)==255){
                    if(safe_access_binimg(i+1,hight)==255){
                        if(safe_access_binimg(i+2,hight)==255){
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

uint8_t line_H(int x){
    for(int i=10+2;i<MT9V03X_H-30;i++){
        if(safe_access_binimg(x,i-2)==0){
            if(safe_access_binimg(x,i-1)==0){
                if(safe_access_binimg(x,i)==255){
                    if(safe_access_binimg(x,i+1)==255){
                        if(safe_access_binimg(x,i+2)==255){
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int element_L(){
    int count = 0;
    count = line_H(25);
    return count;
}

int element_R(){
    return line_H(MT9V03X_W-25);
}

int element_U(){
    return line_W(10);
}

int element_B(){
    return line_W(MT9V03X_H-40);
}

int element_M(){
    return line_W(MT9V03X_H-20);
}

void show_element_line(){
    //ips200_show_int(0,140,element_line_M,2);
    ips200_show_int(0,160,element_line_L,2);
    ips200_show_int(0,180,element_line_R,2);
    ips200_show_int(0,200,element_line_U,2);
    ips200_show_int(0,200,element_line_M,2);
}

void element_update(){
    element_line_L = element_L();
    element_line_R = element_R();
    element_line_U = element_U();
    element_line_M = element_M();
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名称：    check_sharp_turn_state
//  说明：        复用双爬虫出口记录；两条爬虫同侧离开且未从顶部离开，且出边点 y >= lookahead_y（不在图像最顶端）才判为直角。
//-------------------------------------------------------------------------------------------------------------------
int check_sharp_turn_state(int lookahead_y) {
    const ExitLayer *left_exit = &exit_record.left[0];
    const ExitLayer *right_exit = &exit_record.right[0];
    int exit_y;

    if (exit_record.top.L_hit || exit_record.top.R_hit) {
        return 0;
    }

    if (left_exit->L_hit && left_exit->R_hit) {
        exit_y = left_exit->L.pt.y < left_exit->R.pt.y
            ? left_exit->L.pt.y : left_exit->R.pt.y;
    }
    else if (right_exit->L_hit && right_exit->R_hit) {
        exit_y = right_exit->L.pt.y < right_exit->R.pt.y
            ? right_exit->L.pt.y : right_exit->R.pt.y;
    }
    else {
        return 0;
    }

    /* 出边点太靠上（拐角还很远）不算直角，避免识别过早；数值越大识别越晚。 */
    if (exit_y < lookahead_y) {
        return 0;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  check_false_fork_by_white — 假三叉过滤
//  从穿出点沿爬线路径向前（step_idx - 20）取参考点，从该点向上扫描白点。
//  如果白点个数 > 20，说明线路仍在延伸，不是真正的三叉路口。
//  返回 1 = 白点过多，判定为假三叉
//  返回 0 = 检查通过
//-------------------------------------------------------------------------------------------------------------------
static int check_false_fork_by_white(Point exit_pt, int exit_step_idx, int is_L_crawler) {
    // 先清零对应侧的计数
    if (is_L_crawler) {
        fork_white_L = 0;
    } else {
        fork_white_R = 0;
    }

    if (exit_step_idx < 20) return 0;

    // 穿出点沿路径向前 20 步的位置（即穿出点之前的第 20 个点）
    Point ref_pt;
    if (is_L_crawler) {
        ref_pt = Side_L[exit_step_idx - 20];
    } else {
        ref_pt = Side_R[exit_step_idx - 20];
    }

    // 从该点向上扫描，统计白点个数
    int white_count = 0;
    for (int y = ref_pt.y; y >= 0; y--) {
        if (safe_access_binimg(ref_pt.x, y) == 255) {
            white_count++;
        }
    }

    // 保存计数供调试显示
    if (is_L_crawler) {
        fork_white_L = white_count;
    } else {
        fork_white_R = white_count;
    }

    if (white_count > 20) {
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  count_valid_exits — 基于爬线出口记录的多层校验
//  数据来源: findline_twoside() 中记录的 exit_record
//-------------------------------------------------------------------------------------------------------------------
int count_valid_exits() {
    reset_fork_debug();
    out_L = (Point){0};
    out_R = (Point){0};
    Point best_L = {0}, best_R = {0};
    int best_step_L = 0, best_step_R = 0;
    int best_src_L = 0, best_src_R = 0;  // 1=上方穿出, 2=左右边穿出
    if(L_point==0||R_point==0){
        fork_dbg_reason = 1;
        return 0;
    }

    // 三层边界从松到紧逐步尝试，任一层通过即可（爬线出口 x 位置决定触发哪些层）
    int src_L = 0, src_R = 0;
    for (int step = 0; step < 3; step++) {
        ExitLayer *ly_left  = &exit_record.left[step];
        ExitLayer *ly_right = &exit_record.right[step];
        ExitLayer *ly_top   = &exit_record.top;

        // TODO: 多次钻进钻出检查暂时关闭，边界附近易误触发

        // 步数靠后过滤：出口点必须位于爬线后段
        int L_thresh = (int)(L_point * EXIT_STEP_RATIO);
        int R_thresh = (int)(R_point * EXIT_STEP_RATIO);
        fork_dbg_layer = step;
        fork_dbg_thresh_L = L_thresh;
        fork_dbg_thresh_R = R_thresh;

        int L_hit_L = ly_left->L_hit  && ly_left->L.step_idx  >= L_thresh && ly_left->L.pt.y > FORK_SIDE_EXIT_MIN_Y;
        int R_hit_L = ly_left->R_hit  && ly_left->R.step_idx  >= R_thresh && ly_left->R.pt.y > FORK_SIDE_EXIT_MIN_Y;
        int L_hit_R = ly_right->L_hit && ly_right->L.step_idx >= L_thresh && ly_right->L.pt.y > FORK_SIDE_EXIT_MIN_Y;
        int R_hit_R = ly_right->R_hit && ly_right->R.step_idx >= R_thresh && ly_right->R.pt.y > FORK_SIDE_EXIT_MIN_Y;
        int L_hit_U = ly_top->L_hit   && ly_top->L.step_idx   >= L_thresh;
        int R_hit_U = ly_top->R_hit   && ly_top->R.step_idx   >= R_thresh;

        // 判定左右归属
        int state_L = (L_hit_L && !R_hit_L) ? 1 : ( (!L_hit_L && R_hit_L) ? 2 : 0 );
        int state_R = (L_hit_R && !R_hit_R) ? 1 : ( (!L_hit_R && R_hit_R) ? 2 : 0 );
        int state_U = (L_hit_U && !R_hit_U) ? 1 : ( (!L_hit_U && R_hit_U) ? 2 : 0 );

        int has_only_L = (state_L == 1) || (state_R == 1) || (state_U == 1);
        int has_only_R = (state_L == 2) || (state_R == 2) || (state_U == 2);
        fork_dbg_has_only_L = has_only_L;
        fork_dbg_has_only_R = has_only_R;
        fork_dbg_hit_left = (L_hit_L || R_hit_L);
        fork_dbg_hit_right = (L_hit_R || R_hit_R);
        fork_dbg_hit_top = (L_hit_U || R_hit_U);

        // 当前层没有同时出现 L-only 和 R-only，尝试更松的下一层
        if (!(has_only_L && has_only_R)) {
            fork_dbg_reason = 2;
            continue;
        }

        Point final_L, final_R;
        int step_L = 0, step_R = 0;
        int boundary_L = FORK_EXIT_BOUNDARY_NONE;
        int boundary_R = FORK_EXIT_BOUNDARY_NONE;
        src_L = 0; src_R = 0;

        if (state_U == 1) {
            final_L = ly_top->L.pt;
            step_L = ly_top->L.step_idx;
            src_L = 1;
            boundary_L = FORK_EXIT_BOUNDARY_TOP;
        }
        else if (state_L == 1) {
            final_L = ly_left->L.pt;
            step_L = ly_left->L.step_idx;
            src_L = 2;
            boundary_L = FORK_EXIT_BOUNDARY_LEFT;
        }
        else {
            final_L = ly_right->L.pt;
            step_L = ly_right->L.step_idx;
            src_L = 2;
            boundary_L = FORK_EXIT_BOUNDARY_RIGHT;
        }

        if (state_U == 2) {
            final_R = ly_top->R.pt;
            step_R = ly_top->R.step_idx;
            src_R = 1;
            boundary_R = FORK_EXIT_BOUNDARY_TOP;
        }
        else if (state_R == 2) {
            final_R = ly_right->R.pt;
            step_R = ly_right->R.step_idx;
            src_R = 2;
            boundary_R = FORK_EXIT_BOUNDARY_RIGHT;
        }
        else {
            final_R = ly_left->R.pt;
            step_R = ly_left->R.step_idx;
            src_R = 2;
            boundary_R = FORK_EXIT_BOUNDARY_LEFT;
        }
        fork_dbg_step_L = step_L;
        fork_dbg_step_R = step_R;
        fork_dbg_src_L = src_L;
        fork_dbg_src_R = src_R;
        fork_dbg_boundary_L = boundary_L;
        fork_dbg_boundary_R = boundary_R;

        /*
        if (!is_legal_fork_exit_pair(boundary_L, boundary_R)) {
            fork_dbg_reason = 6;
            continue;
        }
        */

        // 高度差与水平差校验
        int y_diff = abs(final_L.y - final_R.y);
        int x_diff = abs(final_L.x - final_R.x);
        fork_dbg_y_diff = y_diff;
        fork_dbg_x_diff = x_diff;
        fork_dbg_geom_ok = 1;

        int hit_Left  = L_hit_L || R_hit_L;
        int hit_Right = L_hit_R || R_hit_R;
        int hit_Top   = L_hit_U || R_hit_U;
        int hit_all_three = (hit_Left && hit_Right && hit_Top);

        if (!hit_all_three) {
            if (src_L == 1 || src_R == 1) {
                if (y_diff <= 10) {
                    fork_dbg_geom_ok = 0;
                    fork_dbg_reason = 3;
                    continue;
                }
                if (x_diff <= 30) {
                    fork_dbg_geom_ok = 0;
                    fork_dbg_reason = 3;
                    continue;
                }
            }
            else if (src_L == 2 && src_R == 2) {
                if (y_diff > 60) {
                    fork_dbg_geom_ok = 0;
                    fork_dbg_reason = 3;
                    continue;
                }
                if (final_L.x > final_R.x) {
                    fork_dbg_geom_ok = 0;
                    fork_dbg_reason = 3;
                    continue;
                }
            }
        }

        // 当前层通过所有校验，记录结果
        best_L = final_L;
        best_R = final_R;
        best_step_L = step_L;
        best_step_R = step_R;
        best_src_L = src_L;
        best_src_R = src_R;
        break;
    }

    // 三层都没有通过 → 不是有效三叉
    if (best_L.x == 0 && best_L.y == 0 && best_R.x == 0 && best_R.y == 0) {
        if (fork_dbg_reason == 0) {
            fork_dbg_reason = 4;
        }
        return 0;
    }

    // 假三叉过滤：只对从左右边界穿出的出口做白点检测
    // 从上方穿出的出口不检测（上方白点无参考意义）
    // best_L 始终来自 L 爬虫，best_R 始终来自 R 爬虫
    if (best_src_L == 2 && check_false_fork_by_white(best_L, best_step_L, 1)) {
        fork_dbg_white_ok = 0;
        fork_dbg_reason = 5;
        return 0;
    }
    if (best_src_R == 2 && check_false_fork_by_white(best_R, best_step_R, 0)) {
        fork_dbg_white_ok = 0;
        fork_dbg_reason = 5;
        return 0;
    }

    out_L = best_L;
    out_R = best_R;
    fork_dbg_reason = 0;
    fork_dbg_white_ok = 1;

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  在图像上绘制出口点（调试用）
//  黄色=边界触达点  白色十字=最终选中的出口点(out_L/out_R)
//-------------------------------------------------------------------------------------------------------------------
void show_exit_points(void) {
    // 找到左右边界被触达的最紧层（0=最紧 x≤11, 2=最松 x≤15）

    int best_left  = -1;
    int best_right = -1;
    for (int i = 0; i < 3; i++) {
        if (best_left  < 0 && (exit_record.left[i].L_hit  || exit_record.left[i].R_hit))  best_left  = i;
        if (best_right < 0 && (exit_record.right[i].L_hit || exit_record.right[i].R_hit)) best_right = i;
    }

    // 只绘制最紧层
    if (best_left >= 0) {
        ExitLayer *ly = &exit_record.left[best_left];
        if (ly->L_hit) {
            int dx = ly->L.pt.x * 135 / MT9V03X_W;
            int dy = ly->L.pt.y * 86 / MT9V03X_H;
            for (int xx = -1; xx <= 1; xx++)
                for (int yy = -1; yy <= 1; yy++) {
                    int px = dx + xx;
                    int py = dy + yy;
                    if (px >= 0 && px < ips114_width_max && py >= 0 && py < ips114_height_max)
                        ips114_draw_point(px, py, RGB565_YELLOW);
                }
        }
        if (ly->R_hit) {
            int dx = ly->R.pt.x * 135 / MT9V03X_W;
            int dy = ly->R.pt.y * 86 / MT9V03X_H;
            for (int xx = -1; xx <= 1; xx++)
                for (int yy = -1; yy <= 1; yy++) {
                    int px = dx + xx;
                    int py = dy + yy;
                    if (px >= 0 && px < ips114_width_max && py >= 0 && py < ips114_height_max)
                        ips114_draw_point(px, py, RGB565_YELLOW);
                }
        }
    }

    if (best_right >= 0) {
        ExitLayer *ly = &exit_record.right[best_right];
        if (ly->L_hit) {
            int dx = ly->L.pt.x * 135 / MT9V03X_W;
            int dy = ly->L.pt.y * 86 / MT9V03X_H;
            for (int xx = -1; xx <= 1; xx++)
                for (int yy = -1; yy <= 1; yy++) {
                    int px = dx + xx;
                    int py = dy + yy;
                    if (px >= 0 && px < ips114_width_max && py >= 0 && py < ips114_height_max)
                        ips114_draw_point(px, py, RGB565_YELLOW);
                }
        }
        if (ly->R_hit) {
            int dx = ly->R.pt.x * 135 / MT9V03X_W;
            int dy = ly->R.pt.y * 86 / MT9V03X_H;
            for (int xx = -1; xx <= 1; xx++)
                for (int yy = -1; yy <= 1; yy++) {
                    int px = dx + xx;
                    int py = dy + yy;
                    if (px >= 0 && px < ips114_width_max && py >= 0 && py < ips114_height_max)
                        ips114_draw_point(px, py, RGB565_YELLOW);
                }
        }
    }

    // 顶部出口
    if (exit_record.top.L_hit) {
        int dx = exit_record.top.L.pt.x * 135 / MT9V03X_W;
        int dy = exit_record.top.L.pt.y * 86 / MT9V03X_H;
        for (int xx = -1; xx <= 1; xx++)
            for (int yy = -1; yy <= 1; yy++) {
                int px = dx + xx;
                int py = dy + yy;
                if (px >= 0 && px < ips114_width_max && py >= 0 && py < ips114_height_max)
                    ips114_draw_point(px, py, RGB565_YELLOW);
            }
    }
    if (exit_record.top.R_hit) {
        int dx = exit_record.top.R.pt.x * 135 / MT9V03X_W;
        int dy = exit_record.top.R.pt.y * 86 / MT9V03X_H;
        for (int xx = -1; xx <= 1; xx++)
            for (int yy = -1; yy <= 1; yy++) {
                int px = dx + xx;
                int py = dy + yy;
                if (px >= 0 && px < ips114_width_max && py >= 0 && py < ips114_height_max)
                    ips114_draw_point(px, py, RGB565_YELLOW);
            }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  在屏幕右侧显示三叉判断关键数据（调试用）
//-------------------------------------------------------------------------------------------------------------------
void show_fork_debug_info(void) {
    // 显示假三叉白点计数值，供调整阈值参考（LCD 240×135）
    // 放在图像区域(135x86)下方，单行并排显示
    ips114_show_string(0, 90, "WL:");
    ips114_show_int(24, 90, fork_white_L, 4);
    ips114_show_string(80, 90, "WR:");
    ips114_show_int(104, 90, fork_white_R, 4);
}
