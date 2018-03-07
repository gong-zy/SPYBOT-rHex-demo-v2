
#ifndef SERVOLIB
#define SERVOLIB

/*
 * set the motor to the position
 * position range: -100:100 (percent)
 */

/*
 * the pulse variables store the pulse with for the servos channel
 */

struct Servos {
	int pulse[6];
	int period;

	Servos(int timerPeriod) {
		period = timerPeriod;

		for (int i = 0; i < 6; i++) {
			pulse[i] = period/2;
		}
	}

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
	void initChannel(int channelnum) {

	}


};

#endif //SERVOLIB.H
