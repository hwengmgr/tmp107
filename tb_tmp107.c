/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * tb_tmp107.c
 *
 *  Created on: Dec 17, 2015
 *      Author: a0271474
 */
 
#include <stdio.h>
#include "tmp107.h"

void TMP107_Transmit(char* tx_data, char tx_size){
	char i;
	printf("0x%02X Calibration Byte\n", 0x55);
	printf("0x%02X Command Byte\n", (unsigned char)tx_data[0]);
	for (i = 1; i < tx_size; i++){
		printf("0x%02X Byte %d\n", (unsigned char)tx_data[i], i);
	}
}

char TMP107_WaitForEcho(char tx_size, char rx_size, int timeout_ms){
}
char TMP107_CheckEcho(char* tx_data, char tx_size){
}
void TMP107_RetrieveReadback(char tx_size, char* rx_data, char rx_size){
}
void TMP107_Wait(char wait_ms){
}

void main(){
	char data[10];
	char size;
	
	printf("GlobalAddressInit:\n");
	TMP107_GlobalAddressInit();
	printf("\n");
	float temp_c;
	temp_c = TMP107_DecodeTemperatureResult(0x1A, 0xB0);
	printf("DecodeTemperatureResult(0x1AB0): %f\n", temp_c);
	printf("\n");
	char addr;
	addr = TMP107_Encode5bitAddress(0x01);
	printf("TMP107_Encode5bitAddress(0x01): 0x%02X\n", (unsigned char)addr);
	addr = TMP107_Encode5bitAddress(0x03);
	printf("TMP107_Encode5bitAddress(0x03): 0x%02X\n", (unsigned char)addr);
	addr = TMP107_Encode5bitAddress(0x1F);
	printf("TMP107_Encode5bitAddress(0x1F): 0x%02X\n", (unsigned char)addr);
	printf("\n");
	addr = TMP107_Decode5bitAddress(0x08);
	printf("TMP107_Encode5bitAddress(0x08): 0x%02X\n", (unsigned char)addr);
	addr = TMP107_Decode5bitAddress(0x13);
	printf("TMP107_Encode5bitAddress(0x13): 0x%02X\n", (unsigned char)addr);
	addr = TMP107_Decode5bitAddress(0xF8);
	printf("TMP107_Encode5bitAddress(0xF8): 0x%02X\n", (unsigned char)addr);
}
