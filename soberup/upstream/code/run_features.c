#include "run_features.h"

uint8_t record_enable = 1U;
uint8_t speed_decision_enable = 0U;
uint8_t fork_correction_enable = 0U;
uint8_t drs_enable = 0U;

void run_features_set_record(uint8_t enabled)
{
    record_enable = (uint8_t)(enabled != 0U);
    if (record_enable)
    {
        speed_decision_enable = 0U;
        fork_correction_enable = 0U;
    }
}

void run_features_set_speed_decision(uint8_t enabled)
{
    speed_decision_enable = (uint8_t)(enabled != 0U);
    if (speed_decision_enable)
    {
        record_enable = 0U;
    }
}

void run_features_set_fork_correction(uint8_t enabled)
{
    fork_correction_enable = (uint8_t)(enabled != 0U);
    if (fork_correction_enable)
    {
        record_enable = 0U;
    }
}

void run_features_set_drs(uint8_t enabled)
{
    drs_enable = (uint8_t)(enabled != 0U);
}

uint8_t run_features_map_enabled(void)
{
    return (uint8_t)(speed_decision_enable || fork_correction_enable);
}
