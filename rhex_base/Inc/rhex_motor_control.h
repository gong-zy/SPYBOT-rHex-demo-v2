
#include "stm32f4xx.h"

#ifndef RHEX_MOTOR_CONTROL_H
#define RHEX_MOTOR_CONTROL_H

#define MOTOR_SPEED_FWD_MAX (65)
#define MOTOR_SPEED_BCK_MAX (25)
#define MOTOR_SPEED_STOP (50)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void PWM_set_pulse(uint8_t pwm_pin, float pulse);

void stop_all_motors(void);

void PWM_start_all(void);
void PWM_stop_all(void);

#endif // RHEX_MOTOR_CONTROL_H
