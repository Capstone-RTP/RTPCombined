/*
 * zeroing.c
 *
 *  Created on: Apr 2, 2023
 *      Author: kylei
 */

#include "zeroing.h"

void GoHome(stepper* motor){
	//stop any current motion
	stopStepper(motor);
	//make stepper think its far from zero
	motor->CurrentPosition = 0xFFFF;
	motor->TargetPosition = 0xFFFF;
	/*make motor go backward "forever"
	Note: make sure there is some mechanism in main()
	to stop otherwise this will go until it breaks something*/
	setTarget(motor, 0xFFFE,0);
}

