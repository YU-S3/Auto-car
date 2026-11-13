#ifndef CODE_TUNING_PRESETS_H_
#define CODE_TUNING_PRESETS_H_

#include "zf_common_typedef.h"

#define TUNING_PRESET_COUNT 5U
#define TUNING_PRESET_WORD_COUNT 12U
#define TUNING_PRESET_STORAGE_WORDS \
    (TUNING_PRESET_COUNT * TUNING_PRESET_WORD_COUNT)

typedef struct {
    uint8 configured;
    float turn_kp;
    float turn_kd;
    float steer_kp2;
    float gyro_kd;
    float td_freq;
    float td_damp;
    float dead_zone;
    float basic_speed;
    int normal_front_point;
    int fork_front_point;
    int fork_exit_lookback_points;
} TuningPreset;

typedef enum {
    TUNING_PRESET_APPLIED = 0,
    TUNING_PRESET_EMPTY,
    TUNING_PRESET_INVALID
} TuningPresetResult;

uint8 tuning_preset_is_configured(uint8 preset_index);
float tuning_preset_basic_speed(uint8 preset_index);
TuningPresetResult tuning_preset_apply(uint8 preset_index);
void tuning_preset_begin_edit(uint8 preset_index, TuningPreset *edit_buffer);
TuningPresetResult tuning_preset_save_and_apply(
    uint8 preset_index,
    const TuningPreset *edit_buffer);
uint8 tuning_presets_load(const float *storage);
void tuning_presets_store(float *storage);

#endif /* CODE_TUNING_PRESETS_H_ */
