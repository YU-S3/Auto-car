#ifndef CODE_ROUTE_PROGRESS_H_
#define CODE_ROUTE_PROGRESS_H_

#include "route_config.h"

#include <stdint.h>

typedef struct
{
    int map_event_first_index;
    int map_event_last_index;
    int fork_index;
    int start_mileage;
    int end_mileage;
    int angle_count_before;
    int window_start;
    int hard_start;
    int hard_end;
    int window_end;
} ForkCorrectionEvent;

typedef enum
{
    FORK_CORR_OUTSIDE = 0,
    FORK_CORR_ENTRY_WINDOW,
    FORK_CORR_HARD_CORE,
    FORK_CORR_EXIT_WINDOW,
    FORK_CORR_FINISHED
} ForkCorrectionPhase;

typedef enum
{
    FORK_CORR_SOURCE_NONE = 0,
    FORK_CORR_SOURCE_VISION,
    FORK_CORR_SOURCE_FORCED,
    FORK_CORR_SOURCE_NORMAL_EXIT
} ForkCorrectionSource;

typedef enum
{
    ROUTE_PROGRESS_BUILD_NOT_READY = 0,
    ROUTE_PROGRESS_BUILD_OK,
    ROUTE_PROGRESS_BUILD_EMPTY,
    ROUTE_PROGRESS_BUILD_MAP_OVERFLOW,
    ROUTE_PROGRESS_BUILD_CAPACITY,
    ROUTE_PROGRESS_BUILD_INVALID_FORK,
    ROUTE_PROGRESS_BUILD_INVALID_INTERVAL,
    ROUTE_PROGRESS_BUILD_OVERLAP
} RouteProgressBuildResult;

extern int fork_entry_shrink_percent;
extern int fork_exit_shrink_percent;
extern int fork_entry_margin_ticks;
extern int fork_exit_margin_ticks;
extern int fork_hard_core_min_ticks;

extern int route_progress_event_count;
extern int route_progress_current_event_index;
extern int route_progress_mileage_snapshot;
extern int route_progress_config_index;
extern int route_progress_sync_fork_before;
extern int route_progress_sync_fork_after;
extern int route_progress_sync_angle_before;
extern int route_progress_sync_angle_after;
extern uint8_t route_progress_valid;
extern ForkCorrectionPhase route_progress_phase;
extern ForkCorrectionSource route_progress_enter_source;
extern ForkCorrectionSource route_progress_exit_source;
extern RouteProgressBuildResult route_progress_build_result;

void route_progress_clear(void);
uint8_t route_progress_build(void);
uint8_t route_progress_rebuild_map(void);
void route_progress_reset_run(void);
void route_progress_begin_frame(void);

int route_progress_config_fork_index(void);
uint8_t route_progress_vision_entry_allowed(void);
uint8_t route_progress_force_enter_requested(void);
uint8_t route_progress_hold_exit(void);
uint8_t route_progress_normal_exit_allowed(void);
uint8_t route_progress_force_exit_requested(void);
uint8_t route_progress_exit_window_just_opened(void);
void route_progress_on_enter(ForkCorrectionSource source);
void route_progress_on_exit(ForkCorrectionSource source);

const ForkCorrectionEvent *route_progress_get_event(int index);
const ForkCorrectionEvent *route_progress_current_event(void);

#endif /* CODE_ROUTE_PROGRESS_H_ */
