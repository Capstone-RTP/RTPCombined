#include "serialFromPC.h"

//char instrBufferElements;
uint8_t newLine = '\n';
uint8_t carriageReturn = '\r';

UART_HandleTypeDef* huart;

void InitSerialFromPC(UART_HandleTypeDef* huartHandler, uint8_t* rxBuffer){
	huart = huartHandler;
	//allocate for buffer
	//usartInputBuffer = (uint8_t*)calloc(INPUT_BUFFER_SIZE,sizeof(uint8_t));
	usartOutputBuffer = rxBuffer;
}

void SendSerialNewLine(){
	HAL_UART_Transmit(huart,&newLine,1,10);
	HAL_UART_Transmit(huart,&carriageReturn,1,10);
}

//void SendSerialIntText(uint16_t message){
//	//get number of characters for message
//	uint16_t length = snprintf(NULL, 0, "%d", message);
//	//copy string into buffer
//	snprintf(usartOutputBuffer, length + 1, "%d", message);
//	//print to usart
//	HAL_UART_Transmit(huart,usartOutputBuffer,length,10);
//	SendSerialNewLine();
//}

void SendSerialChar(uint8_t c){
	HAL_UART_Transmit(huart,&c,1,10);
}

void SendSerialInt(uint16_t i){
	HAL_UART_Transmit(huart,(uint8_t*)(&i),sizeof(uint16_t),10);
}

void ParseInstructions(uint8_t* rxBuffer, Instruction* i){
	//assume that the characters were sent r,th,y

	uint16_t temp[3];

	for(int a = 0; a < 3; a++){
		temp[a] = rxBuffer[2*a] | (rxBuffer[2*a+1] << 8);
	}

	i->r = temp[0];
	i->th = temp[1];
	i->y = temp[2];
}

/*
void InitInstructionBuffer(){
	instrBufferElements = 0;
	instrBufferLastIndex = (*Instruction)(INSTR_BUFFER_SIZE);
	instrStart = instrBuffer;
	instrEnd = instrBuffer;
}
*/

/*
*Instruction ReadInstructionBuffer(){
	*Instruction i = start;
	instrStart++;
	if(instrStart > instrBuffer + (*Instruction)){
		instrStart = instrBuffer;
	}
	
	instrBufferElements--;
	return i;
}

void WriteInstructionBuffer(Instruction i){
	instrEnd++
	
}
*/
