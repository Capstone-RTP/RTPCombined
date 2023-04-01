/*
 * serialFromPC.h
 *
 *  Created on: Mar 18, 2023
 *      Author: Kyle
 */
 
#ifndef INC_SERIALFROMPC_H
#define INC_SERIALFROMPC_H

//definitions
#define INSTR_BUFFER_SIZE 32
#define OUTPUT_BUFFER_SIZE 64
#define INPUT_BUFFER_SIZE 64

//includes
#include <stdint.h>
#include <stdio.h>
#include "main.h"

//type definitions 
typedef struct Instruction{
	uint16_t r;
	uint16_t th;
	uint16_t y;
} Instruction;

//variable definitions
static uint8_t *usartOutputBuffer;

//Instruction instrBuffer[INSTR_BUFFER_SIZE];
//Instruction* instrStart,instrEnd,instrBufferLastIndex;

//function prototypes
void InitSerialFromPC(UART_HandleTypeDef* huartHandler, uint8_t* rxBuffer);
void SendSerialIntText(uint16_t message);
void SendSerialNewLine();
void SendSerialChar(uint8_t c);
void SendSerialInt(uint16_t i);
void ParseInstructions(uint8_t* rxBuffer, Instruction* i);

#endif
 
