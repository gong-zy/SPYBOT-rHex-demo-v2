
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
