
#include "servolib.h"


void initServos(struct Servos* s, TIM_HandleTypeDef* tim1, TIM_HandleTypeDef* tim2, int timerPeriod) {
	s->period = timerPeriod;

	for (int i = 0; i < 6; i++) {
		s->pulse[i] = s->period/2;
	}

	s->tim1 = tim1;
	s->tim2 = tim2;
}


void initChannel(struct Servos* s, int channelnum, float percent) {

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	switch (channelnum) {
		case 0: {
			// init motor 0
			s->pulse[0] = percent * s->period;
			sConfigOC.Pulse = s->pulse[0];
			HAL_TIM_PWM_ConfigChannel(s->tim1, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(s->tim1, 1);
			break;
		}
		case 1: {
			// init motor 1
			s->pulse[1] = percent * s->period;
			sConfigOC.Pulse = s->pulse[1];
			HAL_TIM_PWM_ConfigChannel(s->tim1, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(s->tim1, 2);
			break;
		}
		case 2: {
			// init motor 2
			s->pulse[2] = percent * s->period;
			sConfigOC.Pulse = s->pulse[2];
			HAL_TIM_PWM_ConfigChannel(s->tim1, &sConfigOC, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(s->tim1, 3);
			break;
		}
		case 3: {
			// init motor 3
			s->pulse[3] = percent * s->period;
			sConfigOC.Pulse = s->pulse[3];
			HAL_TIM_PWM_ConfigChannel(s->tim2, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(s->tim2, 1);
			break;
		}
		case 4: {
			// init motor 4
			s->pulse[4] = percent * s->period;
			sConfigOC.Pulse = s->pulse[4];
			HAL_TIM_PWM_ConfigChannel(s->tim2, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(s->tim2, 2);
			break;
		}
		case 5: {
			// init motor 5
			s->pulse[5] = percent * s->period;
			sConfigOC.Pulse = s->pulse[5];
			HAL_TIM_PWM_ConfigChannel(s->tim2, &sConfigOC, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(s->tim2, 3);
			break;
		}
	}
}
