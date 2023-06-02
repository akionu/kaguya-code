// This code is based on ST's VL53L5CX ULD examples
// which is under BSD 3-clause "New" or "Revised" License(shown at the last part of this file)

/************************************************************/
/*  VL53L5CX ULD motion indicator with detection thresholds */
/************************************************************/
/*
* This example shows how to use the motion indicator with detection threshold.
* This kind of configuration might be used for user detection applications.
* To use this example, user needs to be sure that macro
* VL53L5CX_DISABLE_MOTION_INDICATOR is NOT enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../api/vl53l5cx_api.h"
#include "../../api/vl53l5cx_plugin_motion_indicator.h"
#include "../../api/vl53l5cx_plugin_detection_thresholds.h"

// change below to your settings
#define I2C_SDA 8
#define I2C_SCL 9
#define INT_PIN 11
i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};

volatile int IntCount = 0;

void irq_callback() {
    gpio_set_irq_enabled(INT_PIN, GPIO_IRQ_EDGE_FALL, false);
    IntCount = 1;
    gpio_set_irq_enabled(INT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

int main(void) {
    // pico settings
    stdio_init_all();
    sleep_ms(3000);

    i2c_init(&vl53l5cx_i2c, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

	gpio_init(INT_PIN);
	gpio_set_dir(INT_PIN, GPIO_IN);
	gpio_pull_up(INT_PIN);

    gpio_set_irq_enabled_with_callback(INT_PIN, GPIO_IRQ_EDGE_FALL, true, &irq_callback);

	/*********************************/
	/*   VL53L5CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, i;
	VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
	VL53L5CX_Motion_Configuration 	motion_config;	/* Motion configuration*/
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	Dev.platform.address = (uint8_t)((VL53L5CX_DEFAULT_I2C_ADDRESS >> 1) & 0xFF);
    Dev.platform.i2c     = &vl53l5cx_i2c; // set i2c bus

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L5CX sensor connected */
	status = vl53l5cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&Dev);
	if(status)
	{
		printf("VL53L5CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);


	/*********************************/
	/*   Program motion indicator    */
	/*********************************/

	/* Create motion indicator with resolution 8x8 */
	status = vl53l5cx_motion_indicator_init(&Dev, &motion_config, VL53L5CX_RESOLUTION_8X8);
	if(status)
	{
		printf("Motion indicator init failed with status : %u\n", status);
		return status;
	}

	/* (Optional) Change the min and max distance used to detect motions. The
	 * difference between min and max must never be >1500mm, and minimum never be <400mm,
	 * otherwise the function below returns error 127 */
	status = vl53l5cx_motion_indicator_set_distance_motion(&Dev, &motion_config, 1000, 2000);
	if(status)
	{
		printf("Motion indicator set distance motion failed with status : %u\n", status);
		return status;
	}

	/* If user want to change the resolution, he also needs to update the motion indicator resolution */
	//status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
	//status = vl53l5cx_motion_indicator_set_resolution(&Dev, &motion_config, VL53L5CX_RESOLUTION_4X4);


	/* Set the device in AUTONOMOUS and set a small integration time to reduce power consumption */
	status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
	status = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 2);
	status = vl53l5cx_set_integration_time_ms(&Dev, 10);


	/*********************************/
	/*  Program detection thresholds */
	/*********************************/

	/* In this example, we want 1 thresholds per zone for a 8x8 resolution */
	/* Create array of thresholds (size cannot be changed) */
	VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];

	/* Set all values to 0 */
	memset(&thresholds, 0, sizeof(thresholds));

	/* Add thresholds for all zones (64 zones in resolution 4x4, or 64 in 8x8) */
	for(i = 0; i < 64; i++){
		thresholds[i].zone_num = i;
		thresholds[i].measurement = VL53L5CX_MOTION_INDICATOR;
		thresholds[i].type = VL53L5CX_GREATER_THAN_MAX_CHECKER;
		thresholds[i].mathematic_operation = VL53L5CX_OPERATION_NONE;

		/* The value 44 is given as example. All motion above 44 will be considered as a movement */
		thresholds[i].param_low_thresh = 44;
		thresholds[i].param_high_thresh = 44;
	}

	/* The last thresholds must be clearly indicated. As we have 64
	 * checkers, the last one is the 63 */
	thresholds[63].zone_num = VL53L5CX_LAST_THRESHOLD | thresholds[63].zone_num;

	/* Send array of thresholds to the sensor */
	vl53l5cx_set_detection_thresholds(&Dev, thresholds);

	/* Enable detection thresholds */
	vl53l5cx_set_detection_thresholds_enable(&Dev, 1);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	//IntCount = 0;
	status = vl53l5cx_start_ranging(&Dev);
	printf("Waiting for a movement into the FOV between 1m and 2m...\n");

	loop = 0;
	while(loop < 10)
	{
		if(IntCount)
		{	
			IntCount = 0;
			vl53l5cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 8x8 mode by default, we have a total
			 * of 64 zones to print. For this example, only the data of first zone are
			 * print */
			for(i = 0; i < 64; i++)
			{
				if(Results.motion_indicator.motion[motion_config.map_id[i]] >= 44)
				{
					printf(" Movement detected in this zone : %3d !\n", i);
				}
			}
			printf("\n");
			loop++;
		}

	}

	status = vl53l5cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}

/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/
