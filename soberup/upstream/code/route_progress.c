#include "route_progress.h"

#include "inertial_navigation.h"
#include "map_event.h"
#include "run_features.h"
#include "trajectory_policy.h"

#include <limits.h>
#include <string.h>

extern int fork_num;
extern int angle_num;
extern int current_fork_type;
extern uint8 subject_id;
extern uint8 route_direction;

int fork_entry_shrink_percent = 40;
int fork_exit_shrink_percent = 20;
int fork_entry_margin_ticks = 5000;
int fork_exit_margin_ticks = 3000;
int fork_hard_core_min_ticks = 1000;

static ForkCorrectionEvent route_progress_events[FORK_CONFIG_CAPACITY];
static uint8_t route_progress_event_committed = 0U;
static uint8_t route_progress_event_synced = 0U;
static uint8_t route_progress_exit_window_opened = 0U;
static ForkCorrectionPhase route_progress_previous_phase = FORK_CORR_OUTSIDE;

int route_progress_event_count = 0;
int route_progress_current_event_index = 0;
int route_progress_mileage_snapshot = 0;
int route_progress_config_index = -1;
int route_progress_sync_fork_before = 0;
int route_progress_sync_fork_after = 0;
int route_progress_sync_angle_before = 0;
int route_progress_sync_angle_after = 0;
uint8_t route_progress_valid = 0U;
ForkCorrectionPhase route_progress_phase = FORK_CORR_OUTSIDE;
ForkCorrectionSource route_progress_enter_source = FORK_CORR_SOURCE_NONE;
ForkCorrectionSource route_progress_exit_source = FORK_CORR_SOURCE_NONE;
RouteProgressBuildResult route_progress_build_result =
    ROUTE_PROGRESS_BUILD_NOT_READY;

static int route_progress_round_ratio(int span, int percent)
{
    int64_t scaled = (int64_t)span * (int64_t)percent;
    return (int)((scaled + 50) / 100);
}

static int route_progress_margin_start(int mileage, int margin)
{
    int64_t result = (int64_t)mileage - (int64_t)margin;
    if (result < 0)
    {
        return 0;
    }
    return result > INT_MAX ? INT_MAX : (int)result;
}

static int route_progress_margin_end(int mileage, int margin)
{
    int64_t result = (int64_t)mileage + (int64_t)margin;
    return result > INT_MAX ? INT_MAX : (int)result;
}

static int route_progress_find_fork(int fork_index)
{
    int i;

    for (i = 0; i < route_progress_event_count; i++)
    {
        if (route_progress_events[i].fork_index == fork_index)
        {
            return i;
        }
    }
    return -1;
}

static uint8_t route_progress_params_valid(void)
{
    return (uint8_t)(
        fork_entry_shrink_percent >= 0 &&
        fork_entry_shrink_percent <= 45 &&
        fork_entry_shrink_percent % 5 == 0 &&
        fork_exit_shrink_percent >= 0 &&
        fork_exit_shrink_percent <= 45 &&
        fork_exit_shrink_percent % 5 == 0 &&
        fork_entry_margin_ticks >= 0 &&
        fork_entry_margin_ticks <= 10000 &&
        fork_entry_margin_ticks % 500 == 0 &&
        fork_exit_margin_ticks >= 0 &&
        fork_exit_margin_ticks <= 10000 &&
        fork_exit_margin_ticks % 500 == 0 &&
        fork_hard_core_min_ticks >= 0 &&
        fork_hard_core_min_ticks <= 10000 &&
        fork_hard_core_min_ticks % 500 == 0);
}

static uint8_t route_progress_calculate_windows(void)
{
    int i;

    if (!route_progress_params_valid())
    {
        route_progress_build_result = ROUTE_PROGRESS_BUILD_INVALID_INTERVAL;
        return 0U;
    }

    for (i = 0; i < route_progress_event_count; i++)
    {
        ForkCorrectionEvent *event = &route_progress_events[i];
        int span = event->end_mileage - event->start_mileage;
        int target_core_length;

        if (span <= 0)
        {
            route_progress_build_result =
                ROUTE_PROGRESS_BUILD_INVALID_INTERVAL;
            return 0U;
        }

        event->window_start = route_progress_margin_start(
            event->start_mileage,
            fork_entry_margin_ticks);
        event->hard_start = event->start_mileage +
            route_progress_round_ratio(span, fork_entry_shrink_percent);
        event->hard_end = event->end_mileage -
            route_progress_round_ratio(span, fork_exit_shrink_percent);
        event->window_end = route_progress_margin_end(
            event->end_mileage,
            fork_exit_margin_ticks);

        target_core_length = fork_hard_core_min_ticks;
        if (target_core_length > span)
        {
            target_core_length = span;
        }
        if (event->hard_end - event->hard_start < target_core_length)
        {
            int center = event->start_mileage + span / 2;
            event->hard_start = center - target_core_length / 2;
            event->hard_end = event->hard_start + target_core_length;
            if (event->hard_start < event->start_mileage)
            {
                event->hard_start = event->start_mileage;
                event->hard_end = event->hard_start + target_core_length;
            }
            if (event->hard_end > event->end_mileage)
            {
                event->hard_end = event->end_mileage;
                event->hard_start = event->hard_end - target_core_length;
            }
        }
    }

    for (i = 0; i + 1 < route_progress_event_count; i++)
    {
        ForkCorrectionEvent *current = &route_progress_events[i];
        ForkCorrectionEvent *next = &route_progress_events[i + 1];
        int gap;
        int boundary;

        if (current->end_mileage > next->start_mileage)
        {
            route_progress_build_result = ROUTE_PROGRESS_BUILD_OVERLAP;
            return 0U;
        }

        gap = next->start_mileage - current->end_mileage;
        boundary = current->end_mileage + gap / 2;
        if (current->window_end > boundary)
        {
            current->window_end = boundary;
        }
        if (next->window_start < boundary)
        {
            next->window_start = boundary;
        }
    }

    for (i = 0; i < route_progress_event_count; i++)
    {
        const ForkCorrectionEvent *event = &route_progress_events[i];
        if (!(event->window_start <= event->start_mileage &&
              event->start_mileage <= event->hard_start &&
              event->hard_start < event->hard_end &&
              event->hard_end <= event->end_mileage &&
              event->end_mileage <= event->window_end))
        {
            route_progress_build_result =
                ROUTE_PROGRESS_BUILD_INVALID_INTERVAL;
            return 0U;
        }
        if (i > 0 &&
            route_progress_events[i - 1].window_end > event->window_start)
        {
            route_progress_build_result = ROUTE_PROGRESS_BUILD_OVERLAP;
            return 0U;
        }
    }

    return 1U;
}

void route_progress_clear(void)
{
    map_event_clear();
    memset(route_progress_events, 0, sizeof(route_progress_events));
    route_progress_event_count = 0;
    route_progress_valid = 0U;
    route_progress_build_result = ROUTE_PROGRESS_BUILD_NOT_READY;
    route_progress_reset_run();
}

uint8_t route_progress_build(void)
{
    uint8_t angle_seen[128] = {0U};
    int angle_count = 0;
    int i;

    memset(route_progress_events, 0, sizeof(route_progress_events));
    route_progress_event_count = 0;
    route_progress_valid = 0U;

    if (map_event_overflow)
    {
        route_progress_build_result = ROUTE_PROGRESS_BUILD_MAP_OVERFLOW;
        route_progress_reset_run();
        return 0U;
    }

    for (i = 0; i < g_map_event_count; i++)
    {
        const MapEvent *map_event = &g_map_events[i];

        if (map_event->type == MAP_EVENT_ANGLE)
        {
            if (map_event->id >= 0 && map_event->id < 128 &&
                !angle_seen[map_event->id])
            {
                angle_seen[map_event->id] = 1U;
                angle_count++;
            }
            continue;
        }

        if (map_event->type == MAP_EVENT_FORK)
        {
            int logical_index;
            ForkCorrectionEvent *event;

            if (!route_fork_index_valid(map_event->id))
            {
                route_progress_build_result =
                    ROUTE_PROGRESS_BUILD_INVALID_FORK;
                route_progress_reset_run();
                return 0U;
            }

            logical_index = route_progress_find_fork(map_event->id);
            if (logical_index < 0)
            {
                if (route_progress_event_count >= FORK_CONFIG_CAPACITY)
                {
                    route_progress_build_result =
                        ROUTE_PROGRESS_BUILD_CAPACITY;
                    route_progress_reset_run();
                    return 0U;
                }

                logical_index = route_progress_event_count++;
                event = &route_progress_events[logical_index];
                event->map_event_first_index = i;
                event->map_event_last_index = i;
                event->fork_index = map_event->id;
                event->start_mileage = map_event->start_mileage;
                event->end_mileage = map_event->end_mileage;
                event->angle_count_before = angle_count;
            }
            else
            {
                event = &route_progress_events[logical_index];
                event->map_event_last_index = i;
                if (map_event->start_mileage < event->start_mileage)
                {
                    event->start_mileage = map_event->start_mileage;
                }
                if (map_event->end_mileage > event->end_mileage)
                {
                    event->end_mileage = map_event->end_mileage;
                }
            }
        }
    }

    if (route_progress_event_count <= 0)
    {
        route_progress_build_result = ROUTE_PROGRESS_BUILD_EMPTY;
        route_progress_reset_run();
        return 0U;
    }

    if (!route_progress_calculate_windows())
    {
        route_progress_reset_run();
        return 0U;
    }

    route_progress_valid = 1U;
    route_progress_build_result = ROUTE_PROGRESS_BUILD_OK;
    route_progress_reset_run();
    return 1U;
}

uint8_t route_progress_rebuild_map(void)
{
    map_event_rebuild();
    return route_progress_build();
}

void route_progress_reset_run(void)
{
    route_progress_current_event_index = 0;
    route_progress_mileage_snapshot = 0;
    route_progress_config_index = -1;
    route_progress_event_committed = 0U;
    route_progress_event_synced = 0U;
    route_progress_exit_window_opened = 0U;
    route_progress_previous_phase = FORK_CORR_OUTSIDE;
    route_progress_phase = FORK_CORR_OUTSIDE;
    route_progress_enter_source = FORK_CORR_SOURCE_NONE;
    route_progress_exit_source = FORK_CORR_SOURCE_NONE;
    route_progress_sync_fork_before = fork_num;
    route_progress_sync_fork_after = fork_num;
    route_progress_sync_angle_before = angle_num;
    route_progress_sync_angle_after = angle_num;
}

void route_progress_begin_frame(void)
{
    const ForkCorrectionEvent *event;
    float mileage = car_pos.mileage_ticks;

    route_progress_mileage_snapshot =
        mileage > 0.0f ? (int)(mileage + 0.5f) : 0;
    route_progress_previous_phase = route_progress_phase;
    route_progress_exit_window_opened = 0U;

    if (!fork_correction_enable)
    {
        route_progress_phase = FORK_CORR_OUTSIDE;
        route_progress_config_index = fork_num;
        return;
    }

    if (!route_progress_valid ||
        route_progress_current_event_index < 0 ||
        route_progress_current_event_index >= route_progress_event_count)
    {
        route_progress_phase = FORK_CORR_FINISHED;
        route_progress_config_index = -1;
        return;
    }

    event = &route_progress_events[route_progress_current_event_index];
    if (route_progress_mileage_snapshot < event->window_start)
    {
        route_progress_phase = FORK_CORR_OUTSIDE;
    }
    else if (route_progress_mileage_snapshot < event->hard_start)
    {
        route_progress_phase = FORK_CORR_ENTRY_WINDOW;
    }
    else if (route_progress_mileage_snapshot < event->hard_end)
    {
        route_progress_phase = FORK_CORR_HARD_CORE;
    }
    else if (route_progress_mileage_snapshot < event->window_end)
    {
        route_progress_phase = FORK_CORR_EXIT_WINDOW;
    }
    else
    {
        route_progress_phase = FORK_CORR_FINISHED;
    }

    if (route_progress_event_committed ||
        route_progress_mileage_snapshot >= event->window_start)
    {
        route_progress_config_index = event->fork_index;
    }
    else
    {
        route_progress_config_index = -1;
    }

    if (!route_progress_event_synced &&
        route_progress_mileage_snapshot >= event->window_start)
    {
        route_progress_sync_fork_before = fork_num;
        route_progress_sync_angle_before = angle_num;
        fork_num = event->fork_index;
        angle_num = event->angle_count_before;
        current_fork_type = route_get_fork_type(
            subject_id, route_direction, event->fork_index);
        route_progress_sync_fork_after = fork_num;
        route_progress_sync_angle_after = angle_num;
        route_progress_event_synced = 1U;
    }

    route_progress_exit_window_opened = (uint8_t)(
        route_progress_previous_phase != FORK_CORR_EXIT_WINDOW &&
        route_progress_phase == FORK_CORR_EXIT_WINDOW);
}

int route_progress_config_fork_index(void)
{
    return fork_correction_enable ? route_progress_config_index : fork_num;
}

uint8_t route_progress_vision_entry_allowed(void)
{
    if (!fork_correction_enable)
    {
        return 1U;
    }
    return (uint8_t)(
        route_progress_valid &&
        !route_progress_event_committed &&
        route_progress_phase == FORK_CORR_ENTRY_WINDOW);
}

uint8_t route_progress_force_enter_requested(void)
{
    const ForkCorrectionEvent *event = route_progress_current_event();
    return (uint8_t)(
        fork_correction_enable &&
        route_progress_valid &&
        event != 0 &&
        !route_progress_event_committed &&
        route_progress_mileage_snapshot >= event->hard_start);
}

uint8_t route_progress_hold_exit(void)
{
    const ForkCorrectionEvent *event = route_progress_current_event();
    return (uint8_t)(
        fork_correction_enable &&
        route_progress_event_committed &&
        event != 0 &&
        route_progress_mileage_snapshot < event->hard_end);
}

uint8_t route_progress_normal_exit_allowed(void)
{
    const ForkCorrectionEvent *event;
    if (!fork_correction_enable)
    {
        return 1U;
    }
    event = route_progress_current_event();
    return (uint8_t)(
        route_progress_event_committed &&
        event != 0 &&
        route_progress_mileage_snapshot >= event->hard_end &&
        route_progress_mileage_snapshot < event->window_end);
}

uint8_t route_progress_force_exit_requested(void)
{
    const ForkCorrectionEvent *event = route_progress_current_event();
    return (uint8_t)(
        fork_correction_enable &&
        route_progress_event_committed &&
        event != 0 &&
        route_progress_mileage_snapshot >= event->window_end);
}

uint8_t route_progress_exit_window_just_opened(void)
{
    return route_progress_exit_window_opened;
}

void route_progress_on_enter(ForkCorrectionSource source)
{
    const ForkCorrectionEvent *event = route_progress_current_event();
    if (!fork_correction_enable || event == 0)
    {
        return;
    }

    route_progress_event_committed = 1U;
    route_progress_config_index = event->fork_index;
    route_progress_enter_source = source;
    route_progress_exit_source = FORK_CORR_SOURCE_NONE;
}

void route_progress_on_exit(ForkCorrectionSource source)
{
    const ForkCorrectionEvent *event = route_progress_current_event();
    if (!fork_correction_enable || event == 0)
    {
        return;
    }

    fork_num = event->fork_index + 1;
    if (fork_num >= FORK_CONFIG_CAPACITY)
    {
        fork_num = FORK_CONFIG_CAPACITY - 1;
    }
    route_progress_exit_source = source;
    route_progress_current_event_index++;
    route_progress_event_committed = 0U;
    route_progress_event_synced = 0U;
    route_progress_config_index = -1;
    route_progress_phase = FORK_CORR_FINISHED;
}

const ForkCorrectionEvent *route_progress_get_event(int index)
{
    if (index < 0 || index >= route_progress_event_count)
    {
        return 0;
    }
    return &route_progress_events[index];
}

const ForkCorrectionEvent *route_progress_current_event(void)
{
    return route_progress_get_event(route_progress_current_event_index);
}
