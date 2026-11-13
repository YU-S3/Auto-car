#ifndef CODE_ROUTE_CONFIG_H_
#define CODE_ROUTE_CONFIG_H_

#include "zf_common_typedef.h"

#define FORK_CONFIG_CAPACITY 100

#define ROUTE_DIRECTION_FORWARD 0U
#define ROUTE_DIRECTION_REVERSE 1U
#define ROUTE_SUBJECT_ENCODER_CAL 4U

#define ROUTE_STOP_FORK_PRELIM_FORWARD 54
#define ROUTE_STOP_FORK_PRELIM_REVERSE 22
#define ROUTE_STOP_FORK_FINAL_FORWARD  39
#define ROUTE_STOP_FORK_FINAL_REVERSE  38

/* 角度观测器实际参与校验的三叉数量：置0禁用该路线观测器。
   Pre Fwd 暂置0：叉20/叉27参考航向与实际入叉不符导致误停车，待修正参考表后恢复52。 */
#define ROUTE_ACTUAL_FORK_PRELIM_FORWARD 0
#define ROUTE_ACTUAL_FORK_PRELIM_REVERSE 0
#define ROUTE_ACTUAL_FORK_FINAL_FORWARD  0
#define ROUTE_ACTUAL_FORK_FINAL_REVERSE  0

#define FORK_TYPE_STRAIGHT 0
#define FORK_TYPE_TURN     1

#define FORK_SEQ_LEFT      0
#define FORK_SEQ_RIGHT     1
#define FORK_SEQ_CROSS     2

#define FORK_EXIT_BOUNDARY_NONE  0
#define FORK_EXIT_BOUNDARY_LEFT  1
#define FORK_EXIT_BOUNDARY_RIGHT 2
#define FORK_EXIT_BOUNDARY_TOP   3

/* 名称中的前、后边界分别属于左爬虫和右爬虫。 */
#define FORK_EXIT_PAIR_INVALID    0
#define FORK_EXIT_PAIR_LEFT_TOP   1
#define FORK_EXIT_PAIR_TOP_RIGHT  2
#define FORK_EXIT_PAIR_LEFT_RIGHT 3

#define FILL_NONE 0
#define FILL_H    1
#define FILL_V    2
#define FILL_HV   3

int route_fork_index_valid(int fork_index);
uint8 route_is_encoder_calibration(uint8 subject_id);
int route_get_actual_fork_count(uint8 subject_id, uint8 route_direction);
int route_fork_entry_angle_valid(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index);
int route_get_fork_entry_angle_deg(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index);
int route_get_fork_type(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index);
int route_get_fork_seq(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index);
uint8 route_classify_fork_exit_pair(int left_boundary, int right_boundary);
uint8 route_get_fork_exit_pair(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index);
int route_fork_exit_pair_matches(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index,
    int left_boundary,
    int right_boundary);
uint8 route_get_fill_mode(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index);
int route_get_stop_fork_num(uint8 subject_id, uint8 route_direction);

#endif /* CODE_ROUTE_CONFIG_H_ */
