/*
 * zeroing.h
 *
 *  Created on: Apr 2, 2023
 *      Author: kylei
 */

#ifndef SRC_ZEROING_H_
#define SRC_ZEROING_H_

//includes
#include "stepperControl.h"

//function prototypes
void GoHome(stepper* motor);
void GoHomeR(stepper* motor);

#endif /* SRC_ZEROING_H_ */
