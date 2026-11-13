#include "tuning_presets.h"

#include "line.h"
#include "menu.h"
#include "menu/menu_params.h"
#include "motor.h"
#include "tool.h"

#include <math.h>

extern SteeringPID turnPID;

static TuningPreset g_tuning_presets[TUNING_PRESET_COUNT];

static uint8 tuning_preset_float_in_range(float value, float minimum, float maximum)
{
    return (uint8)(!isnan(value) && value >= minimum && value <= maximum);
}

static uint8 tuning_preset_is_valid(const TuningPreset *preset)
{
    if (preset == NULL)
    {
        return 0U;
    }

    return (uint8)(
        tuning_preset_float_in_range(preset->turn_kp, 0.0f, 2000.0f) &&
        tuning_preset_float_in_range(preset->turn_kd, 0.0f, 2000.0f) &&
        tuning_preset_float_in_range(preset->steer_kp2, 0.0f, 2000.0f) &&
        tuning_preset_float_in_range(preset->gyro_kd, 0.0f, 2000.0f) &&
        tuning_preset_float_in_range(preset->td_freq, 1.0f, 100.0f) &&
        tuning_preset_float_in_range(preset->td_damp, 0.5f, 2.0f) &&
        tuning_preset_float_in_range(preset->dead_zone, 0.0f, 50.0f) &&
        tuning_preset_float_in_range(preset->basic_speed, 0.0f, 2000.0f) &&
        preset->normal_front_point >= 10 && preset->normal_front_point <= 300 &&
        preset->fork_front_point >= 10 && preset->fork_front_point <= 300 &&
        preset->fork_exit_lookback_points >= 0 &&
        preset->fork_exit_lookback_points <= 40);
}

static void tuning_preset_clear(TuningPreset *preset)
{
    if (preset == NULL)
    {
        return;
    }

    preset->configured = 0U;
    preset->turn_kp = 0.0f;
    preset->turn_kd = 0.0f;
    preset->steer_kp2 = 0.0f;
    preset->gyro_kd = 0.0f;
    preset->td_freq = 0.0f;
    preset->td_damp = 0.0f;
    preset->dead_zone = 0.0f;
    preset->basic_speed = 0.0f;
    preset->normal_front_point = 0;
    preset->fork_front_point = 0;
    preset->fork_exit_lookback_points = 0;
}

static void tuning_preset_capture_current(TuningPreset *preset)
{
    if (preset == NULL)
    {
        return;
    }

    preset->configured = 1U;
    preset->turn_kp = turnPID.Kp;
    preset->turn_kd = turnPID.Kd;
    preset->steer_kp2 = steer_Kp2;
    preset->gyro_kd = turnPID.GKD;
    preset->td_freq = td_freq;
    preset->td_damp = td_damp;
    preset->dead_zone = dead_zone;
    preset->basic_speed = basic_speed;
    preset->normal_front_point = normal_front_point;
    preset->fork_front_point = fork_front_point;
    preset->fork_exit_lookback_points = fork_exit_lookback_points;
}

static void tuning_preset_apply_values(const TuningPreset *preset)
{
    turnPID.Kp = preset->turn_kp;
    turnPID.Kd = preset->turn_kd;
    turnPID.GKD = preset->gyro_kd;
    steer_Kp2 = preset->steer_kp2;
    td_freq = preset->td_freq;
    td_damp = preset->td_damp;
    dead_zone = preset->dead_zone;
    basic_speed = preset->basic_speed;
    steer_output_max = basic_speed + 200.0f;
    encoder_max_val = 2 * (int)basic_speed;
    normal_front_point = preset->normal_front_point;
    fork_front_point = preset->fork_front_point;
    fork_exit_lookback_points = preset->fork_exit_lookback_points;
    front_point = normal_front_point;
}

uint8 tuning_preset_is_configured(uint8 preset_index)
{
    if (preset_index >= TUNING_PRESET_COUNT)
    {
        return 0U;
    }
    return g_tuning_presets[preset_index].configured;
}

float tuning_preset_basic_speed(uint8 preset_index)
{
    if (preset_index >= TUNING_PRESET_COUNT)
    {
        return 0.0f;
    }
    return g_tuning_presets[preset_index].basic_speed;
}

TuningPresetResult tuning_preset_apply(uint8 preset_index)
{
    const TuningPreset *preset;

    if (preset_index >= TUNING_PRESET_COUNT)
    {
        return TUNING_PRESET_INVALID;
    }

    preset = &g_tuning_presets[preset_index];
    if (!preset->configured)
    {
        return TUNING_PRESET_EMPTY;
    }
    if (!tuning_preset_is_valid(preset))
    {
        return TUNING_PRESET_INVALID;
    }

    tuning_preset_apply_values(preset);
    menu_params_save_tuning_preset();
    return TUNING_PRESET_APPLIED;
}

void tuning_preset_begin_edit(uint8 preset_index, TuningPreset *edit_buffer)
{
    if (edit_buffer == NULL)
    {
        return;
    }

    if (preset_index < TUNING_PRESET_COUNT &&
        g_tuning_presets[preset_index].configured)
    {
        *edit_buffer = g_tuning_presets[preset_index];
    }
    else
    {
        tuning_preset_capture_current(edit_buffer);
    }
}

TuningPresetResult tuning_preset_save_and_apply(
    uint8 preset_index,
    const TuningPreset *edit_buffer)
{
    TuningPreset saved;

    if (preset_index >= TUNING_PRESET_COUNT || edit_buffer == NULL)
    {
        return TUNING_PRESET_INVALID;
    }

    saved = *edit_buffer;
    saved.configured = 1U;
    if (!tuning_preset_is_valid(&saved))
    {
        return TUNING_PRESET_INVALID;
    }

    g_tuning_presets[preset_index] = saved;
    tuning_preset_apply_values(&saved);
    menu_params_save_tuning_preset();
    return TUNING_PRESET_APPLIED;
}

uint8 tuning_presets_load(const float *storage)
{
    uint8 i;
    uint8 normalized = 0U;

    if (storage == NULL)
    {
        return 1U;
    }

    for (i = 0U; i < TUNING_PRESET_COUNT; i++)
    {
        uint16 offset = (uint16)i * TUNING_PRESET_WORD_COUNT;
        TuningPreset loaded;
        float normal_front = storage[offset + 9U];
        float fork_front = storage[offset + 10U];
        float fork_exit_lookback = storage[offset + 11U];

        tuning_preset_clear(&loaded);
        if (storage[offset] == 0.0f)
        {
            g_tuning_presets[i] = loaded;
            continue;
        }

        if (storage[offset] != 1.0f || isnan(normal_front) ||
            isnan(fork_front) || isnan(fork_exit_lookback) ||
            normal_front < 10.0f ||
            normal_front > 300.0f || fork_front < 10.0f ||
            fork_front > 300.0f || fork_exit_lookback < 0.0f ||
            fork_exit_lookback > 40.0f)
        {
            g_tuning_presets[i] = loaded;
            normalized = 1U;
            continue;
        }

        loaded.configured = 1U;
        loaded.turn_kp = storage[offset + 1U];
        loaded.turn_kd = storage[offset + 2U];
        loaded.steer_kp2 = storage[offset + 3U];
        loaded.gyro_kd = storage[offset + 4U];
        loaded.td_freq = storage[offset + 5U];
        loaded.td_damp = storage[offset + 6U];
        loaded.dead_zone = storage[offset + 7U];
        loaded.basic_speed = storage[offset + 8U];
        loaded.normal_front_point = (int)normal_front;
        loaded.fork_front_point = (int)fork_front;
        loaded.fork_exit_lookback_points = (int)fork_exit_lookback;

        if (!tuning_preset_is_valid(&loaded))
        {
            tuning_preset_clear(&loaded);
            normalized = 1U;
        }
        g_tuning_presets[i] = loaded;
    }

    return normalized;
}

void tuning_presets_store(float *storage)
{
    uint8 i;

    if (storage == NULL)
    {
        return;
    }

    for (i = 0U; i < TUNING_PRESET_COUNT; i++)
    {
        uint16 offset = (uint16)i * TUNING_PRESET_WORD_COUNT;
        const TuningPreset *preset = &g_tuning_presets[i];

        storage[offset] = preset->configured ? 1.0f : 0.0f;
        storage[offset + 1U] = preset->configured ? preset->turn_kp : 0.0f;
        storage[offset + 2U] = preset->configured ? preset->turn_kd : 0.0f;
        storage[offset + 3U] = preset->configured ? preset->steer_kp2 : 0.0f;
        storage[offset + 4U] = preset->configured ? preset->gyro_kd : 0.0f;
        storage[offset + 5U] = preset->configured ? preset->td_freq : 0.0f;
        storage[offset + 6U] = preset->configured ? preset->td_damp : 0.0f;
        storage[offset + 7U] = preset->configured ? preset->dead_zone : 0.0f;
        storage[offset + 8U] = preset->configured ? preset->basic_speed : 0.0f;
        storage[offset + 9U] = preset->configured
            ? (float)preset->normal_front_point : 0.0f;
        storage[offset + 10U] = preset->configured
            ? (float)preset->fork_front_point : 0.0f;
        storage[offset + 11U] = preset->configured
            ? (float)preset->fork_exit_lookback_points : 0.0f;
    }
}
