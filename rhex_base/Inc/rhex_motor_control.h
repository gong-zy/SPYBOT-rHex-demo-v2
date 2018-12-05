
#include "stm32f4xx.h"

#ifndef RHEX_MOTOR_CONTROL_H
#define RHEX_MOTOR_CONTROL_H

#define MOTOR_SPEED_FWD_MAX (65)
#define MOTOR_SPEED_BCK_MAX (25)
#define MOTOR_SPEED_STOP (50)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/*
 * Enum for leg state detection
 */
typedef enum LEG_STATE {
	UP = 0,
	DOWN = 1,
	UNKNOWN = 2
}LEG_STATE;

void PWM_set_pulse(uint8_t pwm_pin, float pulse);

void stop_all_motors(void);

void PWM_start_all(void);
void PWM_stop_all(void);

uint32_t motors_at_state(LEG_STATE* motors, LEG_STATE* states);
LEG_STATE get_motor_state(uint32_t hal_upper, uint32_t hal_lower);
void update_motors_states(LEG_STATE* motors);
void set_states(LEG_STATE* src, LEG_STATE* dest);

#endif // RHEX_MOTOR_CONTROL_H
