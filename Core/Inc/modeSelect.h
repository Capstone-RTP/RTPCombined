/*
 * modeSelect.h
 *
 *  Created on: Apr. 3, 2023
 *      Author: kylei
 */

#ifndef INC_MODESELECT_H_
#define INC_MODESELECT_H_

#define LED_CONFIG_0 0x0
#define LED_CONFIG_1 0x1
#define LED_CONFIG_2 0x2
#define LED_CONFIG_3 0x4
#define LED_CONFIG_4 0x8

typedef enum RTP_MODE{
	RTP_STANDBY,
	RTP_ZERO,
	RTP_TATTOO,
	RTP_SCAN
}RTP_MODE;

//NOTE: using enum instead of int incase we want to use specific names later
typedef enum ZERO_MODE{
	ZERO_Q0,
	ZERO_Q1,
	ZERO_Q2,
	ZERO_Q3,
	ZERO_Q4,
	ZERO_Q5,
	ZERO_Q6
}ZERO_MODE;



#endif /* INC_MODESELECT_H_ */
