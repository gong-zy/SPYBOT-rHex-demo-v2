
#include "stm32f4xx.h"
#include "motor_control.h"



/*
 * Brief: Sets pulse with of a specified PWM channel
 * @pwm_pin: the number of pin
 * @pulse: the puls width in percentage
 */
void PWM_set_pulse(uint8_t pwm_pin, float pulse) {
	pulse = (pulse*50000)/100;

	switch (pwm_pin) {
		case 1: {
			TIM4->CCR1 = pulse;
			break;
		}
		case 2: {
			TIM4->CCR2 = pulse;
			break;
		}
		case 3: {
			TIM4->CCR3 = pulse;
			break;
		}
		case 4: {
			TIM3->CCR1 = pulse;
			break;
		}
		case 5: {
			TIM3->CCR2 = pulse;
			break;
		}
		case 6: {
			TIM3->CCR3 = pulse;
			break;
		}
		default: {
			// invalid pwm_pin
			break;
		}
	}
}

/*
 * Brief: Enables PWM generation for all channels
 */
void PWM_start_all(void) {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/*
 * Brief: Disables PWM generation for all channels
 */
void PWM_stop_all(void) {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}





