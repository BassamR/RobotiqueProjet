/*
* @name		communications.h
* @author 	Micro-315 team
* @date 	Mai 2022
*/

#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H


void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);


#endif /* COMMUNICATIONS_H */
