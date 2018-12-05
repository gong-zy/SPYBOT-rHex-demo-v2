
#include <rhex_motor_control.h>

/*
 * Brief: Sets pulse with of a specified PWM channel
 * @pwm_pin: the number of pin
 * @pulse: the puls width in percentage
 */
void PWM_set_pulse(uint8_t pwm_pin, float pulse) {
	/*
	 * TIM3-4 base clk is 50/3 MHz
	 * 50 000 period is 3 ms
	 * dt clk is 6*10e-8
	 * 0.5 ms is 8333  tick
	 * 2.5 ms is 41667 tick
	 *
	 * 0%	-> 8333
	 * 100%	-> 41667
	 */
	pulse = 8333 + (33334/100)*pulse;
	// lehet hogy ez period-pulse kene hogy legyen
	// lenyegeben invertalva kene kiadni a jelet

	switch (pwm_pin) {
		case 0:
			TIM4->CCR1 = pulse;
			break;

		case 1:
			TIM4->CCR2 = pulse;
			break;

		case 2:
			TIM4->CCR3 = pulse;
			break;

		case 3:
			TIM3->CCR1 = pulse;
			break;

		case 4:
			TIM3->CCR2 = pulse;
			break;

		case 5:
			TIM3->CCR3 = pulse;
			break;

		default:
			// invalid pwm_pin
			break;
	}
}

/*
 * Brief: Stops all motors by setting the pulse with to 50%
 */
void stop_all_motors(void) {
	for (int i = 0; i < 6; i++) {
		PWM_set_pulse(i, MOTOR_SPEED_STOP);
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

/*
 * Brief: Updates te motors position state 
 * @motors: pointer to the motors state array
 * @motors: pointer to the desired positions
 */
uint32_t motors_at_state(LEG_STATE* motors, LEG_STATE* states) {
	for (int i = 0; i < 6; i++) {
		if (motors[i] != states[i]) {
			return 0;
		}
	}
	return 1;
}

/*
 * Brief: Return the state of the motors position
 * @hal_upper: the value of the upper sersors input pin
 * @hal_upper: the value of the lower sersors input pin
 */
LEG_STATE get_motor_state(uint32_t hal_upper, uint32_t hal_lower) {
	if (hal_upper == 0) {
		return UP;
	}
	if (hal_lower == 0) {
		return DOWN;
	}
	return UNKNOWN;
}

/*
 * Brief: Updates te motors position state 
 * @motors: pointer to the motors state array
 */
void update_motors_states(LEG_STATE* motors) {
	motors[0] = get_motor_state(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2), HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6));
	motors[1] = get_motor_state(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2));
	motors[2] = get_motor_state(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3), HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));
	motors[3] = get_motor_state(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
	motors[4] = get_motor_state(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2), HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3));
	motors[5] = get_motor_state(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9), HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0));
}

void set_states(LEG_STATE* src, LEG_STATE* dest) {
	for (int i = 0; i < 6; i++) {
		dest[i] = src[i];
	}
}

void wait_for_state(LEG_STATE* motors, LEG_STATE* states) {
    while (motors_at_state(motors, states) != 1) {
    	update_motors_states(motors);
    	for (int i = 0; i < 6; i++) {
    		if (motors[i] == states[i]) {
    			PWM_set_pulse(i, MOTOR_SPEED_STOP);
    		}
    	}
    }
}

void all_fwd() {
	PWM_set_pulse(0, MOTOR_SPEED_FWD_MAX);
	PWM_set_pulse(1, MOTOR_SPEED_FWD_MAX);
	PWM_set_pulse(2, MOTOR_SPEED_FWD_MAX);
	PWM_set_pulse(3, MOTOR_SPEED_BCK_MAX);
	PWM_set_pulse(4, MOTOR_SPEED_BCK_MAX);
	PWM_set_pulse(5, MOTOR_SPEED_BCK_MAX);
}
