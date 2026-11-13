#ifndef CODE_LINE_H_
#define CODE_LINE_H_

#ifndef M_PI
#define M_PI 3.1415926535f
#endif
extern int fork_num;

void caculate_error();
int return_error();
void show_line();
void show_middle_line();
int get_current_fork_seq(int index);
void show_target_point();

extern int front_point;
extern int normal_front_point;
extern int fork_front_point;
extern int fork_exit_lookback_points;
extern int L_R_difference;


#endif /* CODE_LINE_H_ */
