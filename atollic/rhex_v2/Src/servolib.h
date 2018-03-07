
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#ifndef SERVOLIB
#define SERVOLIB

/*
 * set the motor to the position
 * position range: -100:100 (percent)
 */

/*
 * the pulse variables store the pulse with for the servos channel
 * the period variable stores the timers initializatons parameter for the timers period
 * tim1 and tim2 stores pointers to the global timer variables used to generate the PWM signals
 */

struct Servos {
	int pulse[6];
	int period;
	TIM_HandleTypeDef *tim1, *tim2;
}Servos;


// initialize the servo structure
void initServos(struct Servos* s, TIM_HandleTypeDef* tim1, TIM_HandleTypeDef* tim2, int timerPeriod);



/*
 * there are 6 channles:
 * timer3: ch 1 2 3
 * timer4: ch 1 2 3
 * channelnum:
 * 0: tm 3 ch 1
 * 1: tm 3 ch 2
 * 2: tm 3 ch 3
 * 3: tm 4 ch 1
 * 4: tm 4 ch 2
 * 5: tm 4 ch 3
 */
void initChannel(struct Servos* s, int channelnum, float percent);





#endif //SERVOLIB.H
