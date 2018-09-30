
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void PWM_start_all(void);
void PWM_stop_all(void);

void PWM_set_pulse(uint8_t pwm_pin, float pulse);


#endif // MOTOR_CONTROL_H
