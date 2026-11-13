#include "stdio.h"
#include "image.h"
#include "zf_device_mt9v03x.h"
#include "string.h"
#include "maze.h"
#include "line.h"
#include "zf_device_ips200.h"
#include "motor.h"
#include "element_line.h"

// --- 新增：访问标记宏定义 ---


const int dir_front[4][2]= {{0,  -1},
                            {1,  0},
                            {0,  1},
                            {-1, 0}};

const int dir_frontleft[4][2] = {{-1, -1},
                                 {1,  -1},
                                 {1,  1},
                                 {-1, 1}};

const int dir_frontright[4][2] ={{1,  -1},
                                 {1,  1},
                                 {-1, 1},
                                 {-1, -1}};

Point StarM;
Point Max_Lx,Max_Ly,Max_Rx,Max_Ry;

#pragma section all "cpu1_dsram"
Point Side_L[300], Side_R[300];
static Point cross_backup_L[300], cross_backup_R[300];
#pragma section all restore

int L_point,R_point;
uint8_t touch = 0;
DistTrend trend;
ExitRecord exit_record;
int count_outside = 0;
int cross_rebuild_result = 0;

#define BAD_LINE_FIXED_DIST_THRES 20.0f
#define BAD_LINE_DIST_RATIO_THRES 0.3f

#define CROSS_SCAN_START_OFFSET 5
#define CROSS_CONFIRM_ROWS 5
#define CROSS_CENTER_SCAN_WIDTH 100
extern Point out_L, out_R;

typedef struct {
    int start_y;
    int min_y;
    uint8_t return_count;
    uint8_t advanced;
    uint8_t backtracked;
} LineBacktrackState;

static void line_backtrack_state_init(LineBacktrackState *state, int start_y)
{
    if (state == NULL) return;
    state->start_y = start_y;
    state->min_y = start_y;
    state->return_count = 0;
    state->advanced = 0;
    state->backtracked = 0;
}

static uint8_t line_backtrack_state_update(LineBacktrackState *state, Point point)
{
    if (state == NULL) return 0;
    if (state->backtracked) return 1;

    if (point.y < state->min_y) {
        state->min_y = point.y;
    }

    if (!state->advanced) {
        if (state->start_y - state->min_y >= GEOMETRIC_BACKTRACK_ADVANCE_MIN) {
            state->advanced = 1;
        }
        return 0;
    }

    if (point.y >= state->start_y - GEOMETRIC_BACKTRACK_RETURN_MARGIN) {
        if (state->return_count < GEOMETRIC_BACKTRACK_CONFIRM_POINTS) {
            state->return_count++;
        }
    } else {
        state->return_count = 0;
    }

    if (state->return_count >= GEOMETRIC_BACKTRACK_CONFIRM_POINTS) {
        state->backtracked = 1;
        return 1;
    }
    return 0;
}

static uint8_t crawlers_touch(Point left, Point right)
{
    int dx = left.x - right.x;
    int dy = left.y - right.y;

    return (uint8_t)(dx * dx + dy * dy <=
                     TOUCH_DISTANCE_PX * TOUCH_DISTANCE_PX);
}

static int find_cross_center_edges_at_row(int y, Point *left, Point *right)
{
    int center_x = MT9V03X_W / 2;
    int min_x = center_x - CROSS_CENTER_SCAN_WIDTH / 2;
    int max_x = center_x + CROSS_CENTER_SCAN_WIDTH / 2 - 1;
    int x;
    int left_found = 0;
    int right_found = 0;

    if (y < 0 || y >= MT9V03X_H) return -1;
    if (min_x < 2) min_x = 2;
    if (max_x > MT9V03X_W - 3) max_x = MT9V03X_W - 3;

    for (x = min_x; x <= max_x; x++) {
        if (safe_access_binimg(x - 2, y) == 0 &&
            safe_access_binimg(x - 1, y) == 0 &&
            safe_access_binimg(x, y) != 0 &&
            safe_access_binimg(x + 1, y) != 0) {
            left->x = x;
            left->y = y;
            left_found = 1;
            break;
        }
    }

    for (x = max_x; x >= min_x; x--) {
        if (safe_access_binimg(x - 1, y) != 0 &&
            safe_access_binimg(x, y) != 0 &&
            safe_access_binimg(x + 1, y) == 0 &&
            safe_access_binimg(x + 2, y) == 0) {
            right->x = x;
            right->y = y;
            right_found = 1;
            break;
        }
    }

    return (left_found && right_found && left->x < right->x) ? 0 : -1;
}

static int cross_edge_pair_is_plausible(Point left, Point right, int reference_width)
{
    int width = right.x - left.x;
    return width >= 4 && width <= reference_width * 2;
}

static int find_persistent_cross_lane(Point *left_start, Point *right_start)
{
    int reference_width = Side_R[0].x - Side_L[0].x;
    int max_center_shift;
    int start_y;
    int y;

    if (reference_width <= 0) return 0;
    max_center_shift = reference_width / 2;
    if (max_center_shift < 2) max_center_shift = 2;
    start_y = (out_L.y < out_R.y ? out_L.y : out_R.y) - CROSS_SCAN_START_OFFSET;

    for (y = start_y; y >= SAFE_MARGIN_Y + CROSS_CONFIRM_ROWS - 1; y--) {
        Point first_left, first_right;
        Point row_left, row_right;
        int previous_center;
        int row_offset;
        int valid = 1;

        if (find_cross_center_edges_at_row(y, &first_left, &first_right) != 0 ||
            !cross_edge_pair_is_plausible(first_left, first_right, reference_width)) {
            continue;
        }

        previous_center = (first_left.x + first_right.x) / 2;
        for (row_offset = 1; row_offset < CROSS_CONFIRM_ROWS; row_offset++) {
            int center;
            if (find_cross_center_edges_at_row(y - row_offset, &row_left, &row_right) != 0 ||
                !cross_edge_pair_is_plausible(row_left, row_right, reference_width)) {
                valid = 0;
                break;
            }
            center = (row_left.x + row_right.x) / 2;
            if (abs(center - previous_center) > max_center_shift) {
                valid = 0;
                break;
            }
            previous_center = center;
        }

        if (valid) {
            *left_start = first_left;
            *right_start = first_right;
            return 1;
        }
    }
    return 0;
}

static int find_upmost_point_index(const Point *side, int point_num)
{
    int best_index;
    int i;

    if (side == NULL || point_num <= 0) return -1;
    best_index = 0;
    for (i = 1; i < point_num; i++) {
        if (side[i].y < side[best_index].y) best_index = i;
    }
    return best_index;
}

int cross_rebuild_edges(void)
{
    int left_num;
    int right_num;
    int left_upmost_index;
    int right_upmost_index;
    int old_left_num = L_point;
    int old_right_num = R_point;
    Point left_start, right_start;

    cross_rebuild_result = -1;
    if (fork_dbg_src_L != 2 || fork_dbg_src_R != 2 ||
        (out_L.x == 0 && out_L.y == 0) || (out_R.x == 0 && out_R.y == 0)) {
        return cross_rebuild_result;
    }
    if (!find_persistent_cross_lane(&left_start, &right_start)) {
        cross_rebuild_result = -2;
        return cross_rebuild_result;
    }

    memcpy(cross_backup_L, Side_L, sizeof(Side_L));
    memcpy(cross_backup_R, Side_R, sizeof(Side_R));

    left_num = 0;
    right_num = 0;
    Side_L[0] = left_start;
    Side_R[0] = right_start;
    memset(&exit_record, 0, sizeof(exit_record));
    touch = 0;
    cross_rebuild_result = -3;
    findline_twoside(left_start, right_start, &left_num, &right_num, 300);
    left_upmost_index = find_upmost_point_index(Side_L, left_num);
    right_upmost_index = find_upmost_point_index(Side_R, right_num);

    if (left_upmost_index < 0 || right_upmost_index < 0 ||
        Side_L[left_upmost_index].y >= left_start.y ||
        Side_R[right_upmost_index].y >= right_start.y) {
        memcpy(Side_L, cross_backup_L, sizeof(Side_L));
        memcpy(Side_R, cross_backup_R, sizeof(Side_R));
        L_point = old_left_num;
        R_point = old_right_num;
        calc_side_distance_trend(L_point, R_point, &trend);
        return cross_rebuild_result;
    }

    L_point = left_upmost_index + 1;
    R_point = right_upmost_index + 1;
    calc_side_distance_trend(L_point, R_point, &trend);
    cross_rebuild_result = 1;
    return cross_rebuild_result;
}

// 记录单层边界进入（首次记录坐标，累计进入次数）
static void record_layer_entry(ExitLayer *layer, Point pt, int step_idx, int is_L) {
    if (is_L) {
        if (layer->L_entries == 0) {
            layer->L_hit = 1;
            layer->L.pt = pt;
            layer->L.step_idx = step_idx;
        }
        layer->L_entries++;
    } else {
        if (layer->R_entries == 0) {
            layer->R_hit = 1;
            layer->R.pt = pt;
            layer->R.step_idx = step_idx;
        }
        layer->R_entries++;
    }
}

static void findline_twoside_guarded(Point L, Point R,
                                     int *Lnum, int *Rnum,
                                     int MAX_step,
                                     LineBacktrackState *L_guard,
                                     LineBacktrackState *R_guard) {

    maze L_maze = {0}, R_maze = {0};
    L_maze.step = *Lnum;
    R_maze.step = *Rnum;

    if (safe_access_binimg(L.x, L.y) == 0 || safe_access_binimg(R.x, R.y) == 0) {
        return;
    }

    uint8_t L_stopped = 0;
    uint8_t R_stopped = 0;

    // 边界进出跟踪（检测反复钻进钻出）
    uint8_t L_in_left = 0, L_in_right = 0, L_in_top = 0;
    uint8_t R_in_left = 0, R_in_right = 0, R_in_top = 0;

    while ((L_maze.step < MAX_step-2 || R_maze.step < MAX_step-2) &&
           (!L_stopped || !R_stopped)) {

        // ---------------- 左边巡线逻辑 ----------------
        if (!L_stopped) {
            uint8_t L_moved = 0;
            // 1. 绝对停止条件（Y边界越界、打圈超限、步数超限）
            if (L.y < SAFE_MARGIN_Y || L.y >= MT9V03X_H ||
                L_maze.turn >= 4 || L_maze.step >= MAX_step-2) {
                L_stopped = 1;
            }
            // 2. 条件停止条件（碰到X左右边界，且前方y-10处没有检测到赛道特征）
            else if ((L.x <= 5 || L.x >= MT9V03X_W-5)&&L_maze.step>80) {
                L_stopped = 1;
            }
            // 3. 正常爬线 或 触边但前方有线(继续沿着画面边缘爬)
            else {
                int front_Lx = L.x + dir_front[L_maze.dir][0];
                int front_Ly = L.y + dir_front[L_maze.dir][1];
                int frontleft_x = L.x + dir_frontleft[L_maze.dir][0];
                int frontleft_y = L.y + dir_frontleft[L_maze.dir][1];

                int front_L_value = safe_access_binimg(front_Lx, front_Ly);
                int frontleft_value = safe_access_binimg(frontleft_x, frontleft_y);

                if (front_L_value == 0) {
                    L_maze.dir = (L_maze.dir + 1) % 4;
                    L_maze.turn++;
                } else if (frontleft_value == 0) {
                    L.x += dir_front[L_maze.dir][0];
                    L.y += dir_front[L_maze.dir][1];
                    Side_L[L_maze.step].x = L.x;
                    Side_L[L_maze.step].y = L.y;
                    L_maze.step++;
                    L_moved = 1;
                    L_maze.turn = 0;
                } else {
                    L.x += dir_frontleft[L_maze.dir][0];
                    L.y += dir_frontleft[L_maze.dir][1];
                    L_maze.dir = (L_maze.dir + 3) % 4;
                    Side_L[L_maze.step].x = L.x;
                    Side_L[L_maze.step].y = L.y;
                    L_maze.step++;
                    L_moved = 1;
                    L_maze.turn = 0;
                }

                if (Max_Lx.x > L.x) {
                    Max_Lx.x = L.x;
                    Max_Lx.y = L.y;
                }
                if (Max_Ly.y > L.y) {
                    Max_Ly.y = L.y;
                    Max_Ly.x = L.x;
                }

                // 记录L爬虫边界进入（边缘→界内跳变，检测反复钻进钻出）
                {
                int now_L_in_left  = (L.x <= 10);
                int now_L_in_right = (L.x >= MT9V03X_W - 10);
                int now_L_in_top   = (L.y <= 15);

                if (now_L_in_left && !L_in_left) {
                    record_layer_entry(&exit_record.left[2], L, L_maze.step, 1);
                    if (L.x <= 13) record_layer_entry(&exit_record.left[1], L, L_maze.step, 1);
                    if (L.x <= 11) record_layer_entry(&exit_record.left[0], L, L_maze.step, 1);
                }
                if (now_L_in_right && !L_in_right) {
                    record_layer_entry(&exit_record.right[2], L, L_maze.step, 1);
                    if (L.x >= MT9V03X_W - 13) record_layer_entry(&exit_record.right[1], L, L_maze.step, 1);
                    if (L.x >= MT9V03X_W - 11) record_layer_entry(&exit_record.right[0], L, L_maze.step, 1);
                }
                if (now_L_in_top && !L_in_top) {
                    record_layer_entry(&exit_record.top, L, L_maze.step, 1);
                }

                L_in_left  = now_L_in_left;
                L_in_right = now_L_in_right;
                L_in_top   = now_L_in_top;
                }

                if (L_moved && line_backtrack_state_update(L_guard, L)) {
                    L_stopped = 1;
                }
            }
        }

        // ---------------- 右边巡线逻辑 ----------------
        if (!R_stopped) {
            uint8_t R_moved = 0;
            // 1. 绝对停止条件（Y边界越界、打圈超限、步数超限）
            if (R.y < SAFE_MARGIN_Y || R.y >= MT9V03X_H ||
                R_maze.turn >= 4 || R_maze.step >= MAX_step-2) {
                R_stopped = 1;
            }
            // 2. 条件停止条件（碰到X左右边界，且前方y-10处没有检测到赛道特征）
            else if ((R.x <= 5 || R.x >= MT9V03X_W-5)&& R_maze.step>80) {
                R_stopped = 1;
            }
            // 3. 正常爬线 或 触边但前方有线(继续沿着画面边缘爬)
            else {
                int front_x = R.x + dir_front[R_maze.dir][0];
                int front_y = R.y + dir_front[R_maze.dir][1];
                int frontright_x = R.x + dir_frontright[R_maze.dir][0];
                int frontright_y = R.y + dir_frontright[R_maze.dir][1];

                int front_value = safe_access_binimg(front_x, front_y);
                int frontright_value = safe_access_binimg(frontright_x, frontright_y);

                if (front_value == 0) {
                    R_maze.dir = (R_maze.dir + 3) % 4;
                    R_maze.turn++;
                } else if (frontright_value == 0) {
                    R.x += dir_front[R_maze.dir][0];
                    R.y += dir_front[R_maze.dir][1];
                    Side_R[R_maze.step].x = R.x;
                    Side_R[R_maze.step].y = R.y;
                    R_maze.step++;
                    R_moved = 1;
                    R_maze.turn = 0;
                } else {
                    R.x += dir_frontright[R_maze.dir][0];
                    R.y += dir_frontright[R_maze.dir][1];
                    R_maze.dir = (R_maze.dir + 1) % 4;
                    Side_R[R_maze.step].x = R.x;
                    Side_R[R_maze.step].y = R.y;
                    R_maze.step++;
                    R_moved = 1;
                    R_maze.turn = 0;
                }

                if (Max_Rx.x < R.x) {
                    Max_Rx.x = R.x;
                    Max_Rx.y = R.y;
                }
                if (Max_Ry.y > R.y) {
                    Max_Ry.y = R.y;
                    Max_Ry.x = R.x;
                }

                // 记录R爬虫边界进入（边缘→界内跳变，检测反复钻进钻出）
                {
                int now_R_in_left  = (R.x <= 10);
                int now_R_in_right = (R.x >= MT9V03X_W - 10);
                int now_R_in_top   = (R.y <= 15);

                if (now_R_in_left && !R_in_left) {
                    record_layer_entry(&exit_record.left[2], R, R_maze.step, 0);
                    if (R.x <= 13) record_layer_entry(&exit_record.left[1], R, R_maze.step, 0);
                    if (R.x <= 11) record_layer_entry(&exit_record.left[0], R, R_maze.step, 0);
                }
                if (now_R_in_right && !R_in_right) {
                    record_layer_entry(&exit_record.right[2], R, R_maze.step, 0);
                    if (R.x >= MT9V03X_W - 13) record_layer_entry(&exit_record.right[1], R, R_maze.step, 0);
                    if (R.x >= MT9V03X_W - 11) record_layer_entry(&exit_record.right[0], R, R_maze.step, 0);
                }
                if (now_R_in_top && !R_in_top) {
                    record_layer_entry(&exit_record.top, R, R_maze.step, 0);
                }

                R_in_left  = now_R_in_left;
                R_in_right = now_R_in_right;
                R_in_top   = now_R_in_top;
                }

                if (R_moved && line_backtrack_state_update(R_guard, R)) {
                    R_stopped = 1;
                }

                if (!L_stopped && !R_stopped && crawlers_touch(L, R)) {
                    touch = 1;
                    break;
                }
            }
        }
    }

    *Lnum = L_maze.step;
    *Rnum = R_maze.step;
}

void findline_twoside(Point L, Point R, int *Lnum, int *Rnum, int MAX_step) {
    findline_twoside_guarded(L, R, Lnum, Rnum, MAX_step, NULL, NULL);
}

uint8_t First_Line_M(uint8_t n,uint8_t y){
    int sum = 0;
    int sum_max = 0;
    StarM.x = MT9V03X_W / 2;
    StarM.y = y;

    for(int i=0;i<MT9V03X_W-1;i++){
        for(int j=y;j>0;j--){
            if(safe_access_binimg(i, j) == 0) break;
            sum++;

        }
        if(sum>sum_max){
            sum_max = sum;
            StarM.x = i;
            StarM.y = y;

        }
        sum = 0;
    }
    Side_L[n].x = StarM.x;
    Side_L[n].y = StarM.y;
    Side_R[n].x = StarM.x;
    Side_R[n].y = StarM.y;

    if(sum_max<1) return 0;


    for(int i=StarM.x;i>2;i--){
        if(safe_access_binimg(i-1,StarM.y)==0){
            if(safe_access_binimg(i-2,StarM.y)==0){
                Side_L[n].x = i;
                break;
            }
        }
    }

    for(int i=StarM.x;i<MT9V03X_W-2;i++){
        if(safe_access_binimg(i+1,StarM.y)==0){
            if(safe_access_binimg(i+2,StarM.y)==0){
                Side_R[n].x = i;
                break;
            }
        }
    }
    return 1;
}

int find_line_edges_at_row(int y, Point *L, Point *R) {
    if (y < 0 || y >= MT9V03X_H) {
        return -1;
    }

    int found_L = 0, found_R = 0;

    for (int x = 22; x < MT9V03X_W - 22; x++) {
        uint8_t p0 = safe_access_binimg(x - 2, y);
        uint8_t p1 = safe_access_binimg(x - 1, y);
        uint8_t p2 = safe_access_binimg(x,     y);
        uint8_t p3 = safe_access_binimg(x + 1, y);

        if (p0 == 0 && p1 == 0 && p2 != 0 && p3 != 0) {
            L->x = x;
            L->y = y;
            found_L = 1;
            break;
        }
    }
    for (int x = MT9V03X_W - 22; x >= 22; x--) {
        uint8_t p0 = safe_access_binimg(x - 1, y);
        uint8_t p1 = safe_access_binimg(x,     y);
        uint8_t p2 = safe_access_binimg(x + 1, y);
        uint8_t p3 = safe_access_binimg(x + 2, y);

        if (p0 != 0 && p1 != 0 && p2 == 0 && p3 == 0) {
            R->x = x;
            R->y = y;
            found_R = 1;
            break;
        }
    }

    if (found_L && found_R && L->x < R->x) {
        return 0;
    } else {
        return -1;
    }
}

int find_outermost_edges_at_row(int y, Point *L, Point *R) {
    if (y < 0 || y >= MT9V03X_H) {
        return -1;
    }

    int found_L = 0, found_R = 0;

    for (int x = 2; x < MT9V03X_W - 2; x++) {
        uint8_t p0 = safe_access_binimg(x + 3, y);
        uint8_t p1 = safe_access_binimg(x + 2, y);
        uint8_t p2 = safe_access_binimg(x,     y);
        uint8_t p3 = safe_access_binimg(x + 1, y);

        if (p2 != 0 && p3 != 0) {
            L->x = x;
            L->y = y;
            found_L = 1;
            break;
        }
    }
    for (int x = MT9V03X_W - 3; x >= 2; x--) {
        uint8_t p0 = safe_access_binimg(x - 1, y);
        uint8_t p1 = safe_access_binimg(x,     y);
        uint8_t p2 = safe_access_binimg(x - 2, y);
        uint8_t p3 = safe_access_binimg(x - 3, y);

        if (p0 != 0 && p1 != 0) {
            R->x = x;
            R->y = y;
            found_R = 1;
            break;
        }
    }

    if (found_L && found_R && L->x < R->x) {
        return 0;
    } else {
        return -1;
    }
}

// 从好的一侧起始点出发，在同行扫描找回另一侧的边线起点（白→黑交界）
// bad_side: 0=左线坏了需重找左起点, 1=右线坏了需重找右起点
// 返回0成功, -1失败
static int find_edge_from_good_side(Point good_start, Point *bad_start, int bad_side) {
    int y = good_start.y;

    if (bad_side == 1) {
        // 右线坏了，从左起始点向右找白→黑交界（线的右边缘）
        for (int x = good_start.x + 1; x < MT9V03X_W - 2; x++) {
            if (safe_access_binimg(x, y) != 0 && safe_access_binimg(x + 1, y) == 0) {
                bad_start->x = x;
                bad_start->y = y;
                return 0;
            }
        }
    } else {
        // 左线坏了，从右起始点向左找白→黑交界（线的左边缘）
        for (int x = good_start.x - 1; x >= 2; x--) {
            if (safe_access_binimg(x, y) != 0 && safe_access_binimg(x - 1, y) == 0) {
                bad_start->x = x;
                bad_start->y = y;
                return 0;
            }
        }
    }
    return -1;
}

int calc_side_distance_trend(int Lnum, int Rnum, DistTrend *result)
{
    if (!result) return -1;

    result->slope       = 0.0f;
    result->avg_dist    = 0.0f;
    result->first_dist  = 0.0f;
    result->last_dist   = 0.0f;
    result->valid_pairs = 0;

    if (Lnum <= 0 || Rnum <= 0) return -1;

    float dist_buf[30];   // 两侧各取最后30对, 最多30对
    int   pair_count = 0;
    float sum_dist   = 0.0f;

    // 各自取最后 30 个点 (近端→远端)，两侧独立计算 start
    int WINDOW = 30;
    int l_start = (Lnum > WINDOW) ? Lnum - WINDOW : 0;
    int r_start = (Rnum > WINDOW) ? Rnum - WINDOW : 0;

    int li = l_start;
    int ri = r_start;
    while (li < Lnum && ri < Rnum)
    {
        float dx = (float)(Side_R[ri].x - Side_L[li].x);
        float dy = (float)(Side_R[ri].y - Side_L[li].y);
        float dist = dx * dx + dy * dy;

        dist_buf[pair_count] = dist;
        sum_dist += dist;
        pair_count++;
        li++;
        ri++;
    }

    if (pair_count < MIN_VALID_PAIRS) return -1;

    // 线性回归计算
    float n      = (float)pair_count;
    float sum_x  = 0.0f;
    float sum_y  = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;

    for (int i = 0; i < pair_count; i++)
    {
        float xi = (float)i;
        float yi = dist_buf[i];
        sum_x  += xi;
        sum_y  += yi;
        sum_xx += xi * xi;
        sum_xy += xi * yi;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    float slope = 0.0f;
    if (denom > 1e-6f || denom < -1e-6f)
    {
        slope = (n * sum_xy - sum_x * sum_y) / denom;
    }

    result->slope       = slope;
    result->avg_dist    = sum_dist / n;
    result->first_dist  = dist_buf[0];
    result->last_dist   = dist_buf[pair_count - 1];
    result->valid_pairs = pair_count;

    return 0;
}

void outside_protect(void){
    int line_lost = (L_point < 10 && R_point < 10);
    int pitch_limit = (fabs(get_pitch()) > 40);

    if(line_lost || pitch_limit){
        if(count_outside>20){
            if(set_f == 1) {   // 只有在真正发车后，才执行强制停车
                if (line_lost && pitch_limit) {
                    car_stop(STOP_REASON_LINE_AND_PITCH);
                } else if (line_lost) {
                    car_stop(STOP_REASON_LINE_LOST);
                } else {
                    car_stop(STOP_REASON_PITCH_LIMIT);
                }
            }

        }else{
            count_outside++;
        }
    }else{
        count_outside = 0;
    }
}


static int line_is_bad(const Point *side,
                       int point_count,
                       uint8_t geometric_backtrack)
{
    if (geometric_backtrack) return 1;

    if (point_count > 10) {
        float dx = (float)(side[point_count - 1].x - side[0].x);
        float dy = (float)(side[point_count - 1].y - side[0].y);
        float start_end_dist = sqrtf(dx * dx + dy * dy);

        if (start_end_dist < BAD_LINE_FIXED_DIST_THRES ||
            start_end_dist < point_count * BAD_LINE_DIST_RATIO_THRES) {
            return 1;
        }
    }
    return 0;
}

void search_line_main(void) {
    uint8_t y = MT9V03X_H - 10;
    int MAX_step = 300;
    int pre_Man_Ly = Max_Ly.y;
    int pre_Man_Ry = Max_Ry.y;
    Max_Lx.x = MT9V03X_W - 1;
    Max_Rx.x = 0;
    Max_Ly.y = MT9V03X_H - 1;
    Max_Ry.y = MT9V03X_H - 1;

    int Lnum = 0, Rnum = 0;
    LineBacktrackState L_guard, R_guard;

    Point init_L, init_R;
    int start_found = -1;
    start_found = find_outermost_edges_at_row(y, &init_L, &init_R);


    if (start_found != 0) {
        for (int i = MT9V03X_H - 10; i > MT9V03X_H - 40; i -= 2) {
            start_found = find_outermost_edges_at_row(i, &init_L, &init_R);
            if (start_found == 0) {
                break;
            }
        }
    }

    if (start_found != 0) {
        L_point = 0;
        R_point = 0;
        outside_protect();
        return;
    }

    Side_L[0] = init_L;
    Side_R[0] = init_R;
    line_backtrack_state_init(&L_guard, init_L.y);
    line_backtrack_state_init(&R_guard, init_R.y);



    touch = 0;
    memset(&exit_record, 0, sizeof(exit_record));
    findline_twoside_guarded(Side_L[0], Side_R[0],
                             &Lnum, &Rnum, MAX_step,
                             &L_guard, &R_guard);

    count_outside = 0;

//    ips114_show_float(0, 132, L_density, 2, 2);
//    ips114_show_float(0, 148, R_density, 2, 2);


    L_point = Lnum;
    R_point = Rnum;
    //ips114_show_float(0,200,trend.slope,3,3);

    if (line_is_bad(Side_L, L_point, L_guard.backtracked)) L_point = 0;
    if (line_is_bad(Side_R, R_point, R_guard.backtracked)) R_point = 0;

    // 烂线恢复: 仅一侧烂线时, 从好的一侧原始起点出发重找坏侧起点, 重新爬线
    if ((L_point == 0 && R_point > 0) || (R_point == 0 && L_point > 0)) {
        int bad_side = (L_point == 0) ? 0 : 1;
        Point good_start = (bad_side == 0) ? init_R : init_L;
        Point new_bad_start;
        if (find_edge_from_good_side(good_start, &new_bad_start, bad_side) == 0) {
            // 重设起点，两边一起重新爬
            if (bad_side == 0) {
                Side_L[0] = new_bad_start;
                Side_R[0] = good_start;
            } else {
                Side_L[0] = good_start;
                Side_R[0] = new_bad_start;
            }
            Lnum = 0; Rnum = 0;
            touch = 0;
            memset(&exit_record, 0, sizeof(exit_record));
            line_backtrack_state_init(&L_guard, Side_L[0].y);
            line_backtrack_state_init(&R_guard, Side_R[0].y);
            findline_twoside_guarded(Side_L[0], Side_R[0],
                                     &Lnum, &Rnum, MAX_step,
                                     &L_guard, &R_guard);
            L_point = Lnum;
            R_point = Rnum;

            // 恢复后两侧都重新校验, 避免另一侧在二次爬线中形成新烂线
            if (line_is_bad(Side_L, L_point, L_guard.backtracked)) L_point = 0;
            if (line_is_bad(Side_R, R_point, R_guard.backtracked)) R_point = 0;
        }
    }

    calc_side_distance_trend(L_point, R_point, &trend);
    outside_protect();

}
