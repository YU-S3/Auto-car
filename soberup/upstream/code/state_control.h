#ifndef CODE_STATE_CONTROL_H_
#define CODE_STATE_CONTROL_H_


typedef enum {
    normal,
    fork_straight,
    fork_turn,
    fork_exit_pending
}state;

typedef enum {
    angle,
    none
}element_state;

extern volatile state car_state;
extern volatile element_state line_state;
extern int fork_entry_element_count;
extern int current_fork_type;
extern int angle_num;
extern int fork_normal_entry_count;
extern int fork_forced_entry_count;
extern int fork_normal_exit_count;
extern int fork_forced_exit_count;

void state_reset_run(void);
void state_update(void);
void state_imu_exit_check_isr(void);
float state_get_turn_damping_scale(void);
void show_state(void);
int fork_turn_guide_is_active(void);


#endif /* CODE_STATE_CONTROL_H_ */
