#ifndef CODE_RUN_FEATURES_H_
#define CODE_RUN_FEATURES_H_

#include <stdint.h>

extern uint8_t record_enable;
extern uint8_t speed_decision_enable;
extern uint8_t fork_correction_enable;
extern uint8_t drs_enable;

void run_features_set_record(uint8_t enabled);
void run_features_set_speed_decision(uint8_t enabled);
void run_features_set_fork_correction(uint8_t enabled);
void run_features_set_drs(uint8_t enabled);
uint8_t run_features_map_enabled(void);

#endif /* CODE_RUN_FEATURES_H_ */
