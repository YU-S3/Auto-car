#ifndef CODE_ELEMENT_LINE_H_
#define CODE_ELEMENT_LINE_H_

#include "route_config.h"

#define EXIT_STEP_RATIO 0.7f  // 出口点步数必须 >= 总步数*比例

extern int element_line_L,element_line_R,element_line_U,element_line_B,element_line_M;
extern int fork_white_L, fork_white_R;
extern int fork_dbg_reason;
extern int fork_dbg_layer;
extern int fork_dbg_has_only_L, fork_dbg_has_only_R;
extern int fork_dbg_hit_left, fork_dbg_hit_right, fork_dbg_hit_top;
extern int fork_dbg_step_L, fork_dbg_step_R;
extern int fork_dbg_thresh_L, fork_dbg_thresh_R;
extern int fork_dbg_y_diff, fork_dbg_x_diff;
extern int fork_dbg_geom_ok, fork_dbg_white_ok;
extern int fork_dbg_src_L, fork_dbg_src_R;
extern int fork_dbg_boundary_L, fork_dbg_boundary_R;
extern int angle_detect_lookahead;  // 直角检测前瞻行：出边点 y 必须 >= 此值才算直角

void element_update();
int count_valid_exits();
int check_sharp_turn_state(int lookahead_y);

void show_exit_points(void);
void show_fork_debug_info(void);

#endif /* CODE_ELEMENT_LINE_H_ */
