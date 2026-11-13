#include "trajectory_policy.h"

uint8_t trajectory_encode_fork(int fork_index)
{
    if (fork_index < 0 || fork_index >= 127)
    {
        return TRAJECTORY_ELEMENT_NONE;
    }

    return (uint8_t)(fork_index + 1);
}

uint8_t trajectory_encode_angle(int angle_index)
{
    if (angle_index < 0 || angle_index > 127)
    {
        return TRAJECTORY_ELEMENT_NONE;
    }

    return (uint8_t)(TRAJECTORY_ELEMENT_ANGLE_MASK | (uint8_t)angle_index);
}

TrajectoryElementType trajectory_element_type(uint8_t element_code)
{
    if (element_code == TRAJECTORY_ELEMENT_NONE)
    {
        return TRAJECTORY_ELEMENT_TYPE_NONE;
    }
    if ((element_code & TRAJECTORY_ELEMENT_ANGLE_MASK) != 0U)
    {
        return TRAJECTORY_ELEMENT_TYPE_ANGLE;
    }

    return TRAJECTORY_ELEMENT_TYPE_FORK;
}

int trajectory_element_id(uint8_t element_code)
{
    TrajectoryElementType type = trajectory_element_type(element_code);

    if (type == TRAJECTORY_ELEMENT_TYPE_NONE)
    {
        return -1;
    }
    if (type == TRAJECTORY_ELEMENT_TYPE_ANGLE)
    {
        return (int)(element_code & TRAJECTORY_ELEMENT_ID_MASK);
    }

    return (int)(element_code & TRAJECTORY_ELEMENT_ID_MASK) - 1;
}

uint16_t trajectory_error_speed_cap(int error, int min_speed, int max_speed)
{
    int error_magnitude;
    int speed_range;
    int speed_cap;

    if (min_speed < 0 || max_speed < min_speed || max_speed > 65535)
    {
        return 0U;
    }
    if (error <= -TRAJECTORY_ERROR_SLOW_FULL ||
        error >= TRAJECTORY_ERROR_SLOW_FULL)
    {
        return (uint16_t)min_speed;
    }
    if (error >= -TRAJECTORY_ERROR_SLOW_START &&
        error <= TRAJECTORY_ERROR_SLOW_START)
    {
        return (uint16_t)max_speed;
    }

    error_magnitude = (error < 0) ? -error : error;
    speed_range = max_speed - min_speed;
    speed_cap = max_speed -
        speed_range * (error_magnitude - TRAJECTORY_ERROR_SLOW_START) /
        (TRAJECTORY_ERROR_SLOW_FULL - TRAJECTORY_ERROR_SLOW_START);

    if (speed_cap < min_speed)
    {
        speed_cap = min_speed;
    }
    else if (speed_cap > max_speed)
    {
        speed_cap = max_speed;
    }

    return (uint16_t)speed_cap;
}
