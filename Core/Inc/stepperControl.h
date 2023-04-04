/*
 * stepperControl.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Kevin
 */

#include "stm32l4xx_hal.h"

#ifndef INC_STEPPERCONTROL_H_
#define INC_STEPPERCONTROL_H_

typedef enum {
	RunningForward =1,
	RunningBackward =2,
	Stopped =3
}stepperStatus;

typedef struct {

	//hardware
	TIM_HandleTypeDef * Timer;
	uint32_t  Channel;
	GPIO_TypeDef * DIRPort;
	uint16_t DIRPin;

	//speed
	volatile uint32_t PPS;
	volatile uint32_t PPS_ZeroDefault;
	volatile uint32_t PPS_TattooDefault;
	volatile uint32_t PPS_ScanDefault;

	//state
	volatile uint64_t CurrentPosition;
	volatile uint64_t TargetPosition;
	volatile stepperStatus Status;

}stepper;

void initStepper(stepper * stepper, TIM_HandleTypeDef * tim, uint32_t  channel, GPIO_TypeDef * dirPort, uint16_t dirPin, uint32_t speed);
void setTarget (stepper * stepper, uint64_t increment, char forward);
void setSpeed(stepper * stepper, uint32_t speed);
void setDefaultSpeed(stepper * stepper, uint32_t zeroSpeed, uint32_t tattooSpeed, uint32_t scanSpeed);
void setDirection(stepper *stepper);
void stopStepper(stepper *stepper);
void zeroStepper(stepper *stepper);



#endif /* INC_STEPPERCONTROL_H_ */
