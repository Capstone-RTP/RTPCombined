#include "stepperControl.h"

void initStepper(stepper * stepper, TIM_HandleTypeDef * tim, uint32_t  channel, GPIO_TypeDef * dirPort, uint16_t dirPin, uint32_t speed){

	//Initialize values
	stepper->Timer = tim;
	stepper->Channel = channel;
	stepper->DIRPort = dirPort;
	stepper->DIRPin = dirPin;
	stepper->PPS = speed;
	stepper->CurrentPosition = 0;
	stepper->TargetPosition =0;
	//Set status
	stepper->Status = Stopped;
	//update timer to align with speed
	setSpeed(stepper, stepper->PPS);
	//return stepper structure pointer

	//set default speeds
	stepper->PPS_ZeroDefault;
	stepper->PPS_TattooDefault;
	stepper->PPS_ScanDefault;
}


void setTarget (stepper * stepper, uint64_t increment, char forward){
	//if positive move forward and add to current position
	if(forward){
		stepper->TargetPosition+= increment;
	}
	//if negative move backward and take away from current position
	else{
		stepper->TargetPosition-= increment;
	}
	//TargetPosition can't be negative
	if(stepper->TargetPosition<0){
		stepper->TargetPosition=0;
	}
	//Start timer if it needs to be started
	if (stepper->Status == Stopped && stepper->TargetPosition != stepper->CurrentPosition){
		//start pulses on timer
		HAL_TIM_PWM_Start(stepper->Timer, stepper->Channel);
	}
	//Set Direction
	setDirection(stepper);
}

void setSpeed (stepper * stepper, uint32_t speed){
	//update pulse rate of stepper
	stepper->PPS = speed;
	//set duty cycle
	switch(stepper->Channel){
		case TIM_CHANNEL_1:{
			stepper->Timer->Instance->CCR1=1000000/(2*speed);
			break;
		}
		case TIM_CHANNEL_2:{
			stepper->Timer->Instance->CCR2=1000000/(2*speed);
			break;
		}
		case TIM_CHANNEL_3:{
			stepper->Timer->Instance->CCR3=1000000/(2*speed);
			break;
		}
		case TIM_CHANNEL_4:{
			stepper->Timer->Instance->CCR4=1000000/(2*speed);
			break;
		}
	}

	//set timer period
	stepper->Timer->Instance->ARR=(1000000/speed)-1;
}

void setDefaultSpeed(stepper * stepper, uint32_t zeroSpeed, uint32_t tattooSpeed, uint32_t scanSpeed){
	stepper->PPS_ZeroDefault;
	stepper->PPS_TattooDefault;
	stepper->PPS_ScanDefault;

}

//set direction
void setDirection(stepper *stepper){
	if(stepper->CurrentPosition > stepper->TargetPosition){
		HAL_GPIO_WritePin(stepper->DIRPort, stepper->DIRPin, SET);
		stepper->Status = RunningBackward;
	}
	else if(stepper->CurrentPosition < stepper->TargetPosition){
		HAL_GPIO_WritePin(stepper->DIRPort, stepper->DIRPin, RESET);
		stepper->Status = RunningForward;
	}
}

void stopStepper(stepper *stepper){
	HAL_TIM_PWM_Stop(stepper->Timer, stepper->Channel);
	HAL_TIM_Base_Stop(stepper->Timer);
	stepper->Status = Stopped;
	stepper->TargetPosition = stepper->CurrentPosition;
}

void zeroStepper(stepper *stepper){
	stopStepper(stepper);
	stepper->CurrentPosition = 0;
	stepper->TargetPosition = 0;
}
