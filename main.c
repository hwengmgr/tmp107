/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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
 * ======== main.c ========
 * TMP107 Library Demo
 *
 * This demo reads a chain of TMP107s and reports temperature over USB
 * Virtual COM port. The output can be viewed with a terminal application
 * such as putty.
 *
 * On the 5529 LaunchPad, push button S1 (P2.1) is used to trigger a
 * Global Address Init of the TMP107 device chain. This is necessary to
 * perform once on a new set of TMP107s, or when the chain is reconfigured.
 */
#include <stdio.h>
#include <string.h>
#include "driverlib.h"
#include "tmp107.h"
#include "hal.h"

void main(void)
{
	char msg[20];
	char tx[8];
	char rx[64];
	char last_addr;
	char i;
	unsigned char device_count;
	//must set printf compile option to full!
	float tmp107_temp;

	WDT_A_hold(WDT_A_BASE); //Stop watchdog timer
	initClocks(8e6);
	initPC_UART();
	initTMP107();
	GPIO_setAsInputPinWithPullUpResistor(2, GPIO_PIN1);

	__enable_interrupt();  // Enable interrupts globally
	while(1){
		/* perform Global Address Initialize
		 * if launchpad button P2.1 pressed
		 */
		if (GPIO_getInputPinValue(2, GPIO_PIN1)) {
			last_addr = TMP107_LastDevicePoll();
		} else {
			last_addr = TMP107_GlobalAddressInit();
		}
		/* since we init the first device in the chain
		 * as address 1, the last address is also device
		 * count. however, the address is in a bit-reversed
		 * format, and this Decode function fixes that.
		 */
		device_count = TMP107_Decode5bitAddress(last_addr);

		// build global temperature read command packet
		tx[0] = TMP107_Global_bit | TMP107_Read_bit | last_addr;
		tx[1] = TMP107_Temp_reg;
		// transmit global temperature read command
		TMP107_Transmit(tx,2);
		/* master cannot transmit again until after we've received
		 * the echo of our transmit and given the TMP107 adequate
		 * time to reply. thus, we wait.
		 */
		TMP107_WaitForEcho(2,device_count*2,TMP107_Timeout);
		// copy the response from TMP107 into user variable
		TMP107_RetrieveReadback(2,rx,device_count*2);

		for (i = 0; i < device_count;i++){
			// convert two bytes received from each TMP107 into degrees C
			tmp107_temp = TMP107_DecodeTemperatureResult(rx[i*2+1],rx[i*2]);
			sprintf(msg, "temp %d: %f\r\n",i,tmp107_temp);
			PC_Transmit(msg,strlen(msg));
		}
		//update twice a minute to let the 9600 baud PC back end catch up
		TMP107_Wait(30000);
	}
}
