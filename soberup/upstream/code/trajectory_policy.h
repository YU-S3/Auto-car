#ifndef CODE_TRAJECTORY_POLICY_H_
#define CODE_TRAJECTORY_POLICY_H_

#include <stdint.h>

#define TRAJECTORY_ELEMENT_NONE       0x00U
#define TRAJECTORY_ELEMENT_ANGLE_MASK 0x80U
#define TRAJECTORY_ELEMENT_ID_MASK    0x7FU

#define TRAJECTORY_ERROR_SLOW_START 15
#define TRAJECTORY_ERROR_SLOW_FULL  45

typedef enum
{
    TRAJECTORY_ELEMENT_TYPE_NONE = 0,
    TRAJECTORY_ELEMENT_TYPE_FORK,
    TRAJECTORY_ELEMENT_TYPE_ANGLE
} TrajectoryElementType;

uint8_t trajectory_encode_fork(int fork_index);
uint8_t trajectory_encode_angle(int angle_index);
TrajectoryElementType trajectory_element_type(uint8_t element_code);
int trajectory_element_id(uint8_t element_code);
uint16_t trajectory_error_speed_cap(int error, int min_speed, int max_speed);

#endif /* CODE_TRAJECTORY_POLICY_H_ */
