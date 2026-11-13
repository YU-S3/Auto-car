/**************************************************************
 * 文件名:  line.c
 * 创建时间: 2026年2月18日 下午3:57:37
 * 说明:
 **************************************************************/
#include "maze.h"
#include "element_line.h"
#include "image.h"
#include "stdio.h"
#include "line.h"
#include "tool.h"
#include "zf_device_ips114.h"
#include "state_control.h"
#include "menu.h"
#include "run_features.h"
#include "route_config.h"
#include "route_progress.h"

#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : (v) > (hi) ? (hi) : (v))

static void draw_thick_point(int x, int y, uint16 color)
{
    int dx;
    int dy;

    for (dx = -1; dx <= 1; dx++) {
        for (dy = -1; dy <= 1; dy++) {
            int point_x = x + dx;
            int point_y = y + dy;

            if (point_x >= 0 && point_x < 135 &&
                point_y >= 0 && point_y < 86) {
                ips114_draw_point((uint16)point_x, (uint16)point_y, color);
            }
        }
    }
}

volatile int error=0;
int last_error = 0;
int L_R_difference = 0;

Point target_L,target_R,target_M;
int fork_num = 0;

int front_point = 95;
int normal_front_point = 85;
int fork_front_point = 125;
int fork_exit_lookback_points = 30;
Point R,L,M;

#define CROSS_ERROR_HOLD_FRAMES 3
static uint8 cross_hold_valid = 0;
static uint8 cross_hold_fail_frames = 0;
static int cross_hold_error = 0;
static int cross_hold_difference = 0;
static Point cross_hold_L, cross_hold_R, cross_hold_M;

static int get_farthest_lookback_point(const Point *side, int point_count,
                                       Point *target)
{
    int target_index = point_count - 1 - fork_exit_lookback_points;

    if (side == NULL || target == NULL ||
        target_index < 0 || target_index >= point_count) {
        return 0;
    }

    *target = side[target_index];
    return target->x != 0 && target->y != 0;
}


// 根据当前科目获取 fork_seq 的值
int get_current_fork_seq(int index) {
    int config_index = route_progress_config_fork_index();
    (void)index;
    return route_get_fork_seq(
        subject_id, route_direction, config_index);
}
//-------------------------------------------------------------------------------------------------------------------
//  函数功能     计算某点与图像中点连线的夹角
//  返回值       int
//  参数说明      x,y 要计算的坐标
//  使用示例      get_angle(M.x,M.y);
//---------------------------------------------------------------------------------------------------------------
int get_angle(int x,int y){
    x = (MT9V03X_W/2)-x;
    y = MT9V03X_H - y;
    int result = (int)(atan2f(x, y) * 180.0f / M_PI);
    return result;
}



//-------------------------------------------------------------------------------------------------------------------
//  函数功能      计算一边界点的平均坐标
//  返回值       Side(包含平均x,y坐标的结构体)
//  参数说明      sidein 边界点数组指针
//              min 起始索引
//              max 结束索引(含)
//  使用示例      Side avg = average_point(Side_L, 10, 20);
//---------------------------------------------------------------------------------------------------------------
Point average_point(Point *Pointin,int min,int max){
    Point sum;
    sum.x = 0;
    sum.y = 0;
    int count = 0;

    for(int i=min;i<max;i++){
        sum.x += Pointin[i].x;
        sum.y += Pointin[i].y;
        count +=  1;
    }

    sum.x = sum.x/count;
    sum.y = sum.y/count;
    return sum;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数功能     在屏幕上可视化显示检测到的边缘线
//---------------------------------------------------------------------------------------------------------------
void show_line(){
    for(int i=0;i<L_point;i++){
        // 将原始坐标映射到显示区域
        int draw_x = Side_L[i].x * 135 / MT9V03X_W;
        int draw_y = Side_L[i].y * 86 / MT9V03X_H;

        // 添加边界保护，确保绘制在屏幕分辨率范围内
        if(draw_x >= 0 && draw_x < ips114_width_max && draw_y >= 0 && draw_y < ips114_height_max){
            draw_thick_point(draw_x, draw_y, RGB565_RED);
        }
    }

    for(int i=0;i<R_point;i++){
        // 将原始坐标映射到显示区域
        int draw_x = Side_R[i].x * 135 / MT9V03X_W;
        int draw_y = Side_R[i].y * 86 / MT9V03X_H;

        // 添加边界保护，确保绘制在屏幕分辨率范围内
        if(draw_x >= 0 && draw_x < ips114_width_max && draw_y >= 0 && draw_y < ips114_height_max){
            draw_thick_point(draw_x, draw_y, RGB565_CYAN);
        }
    }

}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     显示中线
//---------------------------------------------------------------------------------------------------------------
void show_middle_line(){
    for(int i=0;i<min(L_point,R_point);i++){
        Point M;
        M.x = (int)((Side_L[i].x+Side_R[i].x)/2);
        M.y = (int)((Side_L[i].y+Side_R[i].y)/2);

        // 判断是否使用原始坐标定义，防止越界
        if(M.x > 0 && M.x < MT9V03X_W && M.y > 0 && M.y < MT9V03X_H){
            // 将中线的原始坐标映射到显示区域
            int draw_x = M.x * 135 / MT9V03X_W;
            int draw_y = M.y * 86 / MT9V03X_H;

            // 添加边界保护，确保绘制在屏幕分辨率范围内
            if(draw_x >= 0 && draw_x < ips114_width_max && draw_y >= 0 && draw_y < ips114_height_max){
                show_point_yellow(draw_x, draw_y);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     显示目标点
//---------------------------------------------------------------------------------------------------------------
void show_target_point(){
    int draw_Mx = M.x * 135 / MT9V03X_W;
    int draw_My = M.y * 86 / MT9V03X_H;

    int draw_Lx = L.x * 135 / MT9V03X_W;
    int draw_Ly = L.y * 86 / MT9V03X_H;

    int draw_Rx = R.x * 135 / MT9V03X_W;
    int draw_Ry = R.y * 86 / MT9V03X_H;

    draw_Mx = CLAMP(draw_Mx, 11, 135 - 12);
    draw_My = CLAMP(draw_My, 11, 86 - 12);
    draw_Lx = CLAMP(draw_Lx, 11, 135 - 12);
    draw_Ly = CLAMP(draw_Ly, 11, 86 - 12);
    draw_Rx = CLAMP(draw_Rx, 11, 135 - 12);
    draw_Ry = CLAMP(draw_Ry, 11, 86 - 12);

    if(car_state == normal || car_state == fork_exit_pending){
        if(L_point == 0 && R_point == 0) return;
        show_point(draw_Mx, draw_My);
    }else{
        int current_seq = get_current_fork_seq(fork_num);

        if(current_seq == 0){
            if(L_point == 0) return;
            show_point(draw_Lx, draw_Ly);
        }else if(current_seq == 1){
            if(R_point == 0) return;
            show_point(draw_Rx, draw_Ry);
        }else{
            if(L_point == 0 && R_point == 0) return;
            show_point(draw_Mx, draw_My);
        }
    }
}
void caculate_error(){
    int average_range = 5;
    int is_cross = (car_state == fork_straight && get_current_fork_seq(fork_num) == 2);
    error=0;
    uint8_t R_fail = 0,L_fail = 0;

    if (is_cross) {
        cross_rebuild_edges();
        if (cross_rebuild_result < 0 && cross_hold_valid) {
            cross_hold_fail_frames++;
            if (cross_hold_fail_frames <= CROSS_ERROR_HOLD_FRAMES) {
                L = cross_hold_L;
                R = cross_hold_R;
                M = cross_hold_M;
                error = cross_hold_error;
                L_R_difference = cross_hold_difference;
                return;
            }
            cross_hold_valid = 0;
        } else if (cross_rebuild_result == 1) {
            cross_hold_fail_frames = 0;
        }
    } else {
        cross_hold_valid = 0;
        cross_hold_fail_frames = 0;
        cross_rebuild_result = 0;
    }

    if(car_state==fork_turn){
        front_point = fork_front_point;
    }
    else if(line_state==angle){
        front_point = min(L_point,R_point);
    }else if(get_current_fork_seq(fork_num)==2&&car_state==fork_straight){
        front_point = 70;
    }
    else{
        front_point = normal_front_point;
    }

    if (L_point>front_point){
        L = average_point(Side_L,front_point-average_range,front_point);

    }else if(L_point>40){
        L = average_point(Side_L,L_point-average_range,L_point);
    }else if(L_point>20){
        L = average_point(Side_L,15,20);
    }else{
        if(L_point>0){
            L = Side_L[L_point-1];
        }else{
            L_fail = 1;
        }
    }
    if (R_point>front_point){
        R = average_point(Side_R,front_point-average_range,front_point);
    }else if(R_point>40){
        R = average_point(Side_R,R_point-average_range,R_point);
    }else if(R_point>20){
        R = average_point(Side_R,15,20);
    }else{
        if(R_point>0){
            R = Side_R[R_point-1];
        }else{
            R_fail = 1;
        }
    }

    if(L_fail && !R_fail) L = R;
    if(R_fail && !L_fail) R = L;

    M.x = (L.x+R.x)/2;
    M.y = (L.y+R.y)/2;
    int M_result = get_angle(M.x,M.y);
    int L_result = get_angle(L.x,L.y);
    int R_result = get_angle(R.x,R.y);

//    if(abs(M_result-last_error)>60){
//        M_result = last_error;
//    }else{
//        last_error = M_result;
//    }
    L_R_difference = L_result-R_result;
    int current_seq = get_current_fork_seq(fork_num);
    Point fork_guide_point = {0};
    int fork40_blind = (subject_id == 2U &&
        route_direction == ROUTE_DIRECTION_FORWARD &&
        !fork_correction_enable && fork_num == 40);
    if(car_state == normal || car_state == fork_exit_pending){
        if(fork40_blind && R_fail==0 && line_state == none){
            error = R_result;
        }else if(R_fail==1){
            error = L_result;
            L_R_difference = 0;
        }else if(L_fail==1){
            error = R_result;
            L_R_difference = 0;
        }else if(current_seq==5){
            error = R_result;
        }else{
            error = M_result;
        }
    }else{
        if(current_seq == 0){
            int guide_valid = 0;
            if(car_state == fork_turn){
                guide_valid = fork_turn_guide_is_active() &&
                              get_farthest_lookback_point(
                                  Side_L, L_point, &fork_guide_point);
            }
            if(guide_valid){
                L = fork_guide_point;
                error = get_angle(fork_guide_point.x, fork_guide_point.y);
            }else{
                error = L_result;
            }
        }else if(current_seq == 1){
            int guide_valid = 0;
            if(car_state == fork_turn){
                guide_valid = fork_turn_guide_is_active() &&
                              get_farthest_lookback_point(
                                  Side_R, R_point, &fork_guide_point);
            }
            if(guide_valid){
                R = fork_guide_point;
                error = get_angle(fork_guide_point.x, fork_guide_point.y);
            }else{
                error = R_result;
            }
        }else{
           error = M_result;
        }
    }

    if (is_cross && cross_rebuild_result == 1) {
        cross_hold_L = L;
        cross_hold_R = R;
        cross_hold_M = M;
        cross_hold_error = error;
        cross_hold_difference = L_R_difference;
        cross_hold_valid = 1;
        cross_hold_fail_frames = 0;
    }

    //ips114_show_int(0,200,L_R_difference,3);

}

int return_error(){
    return error;
}
