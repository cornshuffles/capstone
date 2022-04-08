/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC4  Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include "boatProtocol.h"
#include <cy_LoRa.h>

#define SAR_INTR_PRIORITY (3U)

// FreeRTOS defines
#define ADC_TASK_PRIORITY   (2)
#define ADC_TASK_STACK_SIZE (1024 * 1)

// FreeRTOS task handles
TaskHandle_t ADC_task_handle;

// Enable FreeRTOS debugging
volatile int uxTopUsedPriority;

volatile uint8_t currentThrottle = 0;
volatile uint8_t currentServoAngle = 0;

void SARCallback(void){
	Cy_SAR_ClearInterrupt(potADC_HW, CY_SAR_INTR_EOS);
	vTaskNotifyGiveFromISR(ADC_task_handle, NULL);
}

void ADC_task(void *arg) {
	while(true) {
		// Wait for notification that a throttle command has been received then set new throttle
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Get results from ADC
		uint16_t throttleCounts = Cy_SAR_GetResult16(potADC_HW, 0);
		uint16_t servoCounts = Cy_SAR_GetResult16(potADC_HW, 1);
		uint16_t throttle_mv = Cy_SAR_CountsTo_mVolts(potADC_HW, 0, throttleCounts);
		uint16_t servo_mv = Cy_SAR_CountsTo_mVolts(potADC_HW, 1, servoCounts);

		// Calculate throttle and servo values
		uint8_t throttle = (uint8_t)((throttle_mv * 0xFF) / CY_CFG_PWR_VDDA_MV);
		uint8_t servoAngle = (servo_mv * 180) / CY_CFG_PWR_VDDA_MV;

		// If the throttle or servo values changed significantly, update them
		if(abs((int16_t)throttle - (int16_t)currentThrottle) > 3){
			currentThrottle = throttle;
			// TODO Send queue message to LoRa task
		}
		if(abs((int16_t)servoAngle - (int16_t)currentServoAngle) > 3){
			currentServoAngle = servoAngle;
			// TODO Send queue message to LoRa task
		}
	}
}

void killTimerCallback(void) {
	// TODO: Send queue to LoRa task
}

//TODO: KillTimer, Button ISRs, LoRa Task

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    // Enable Debug UART
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    // Starting print
    printf("\x1b[2J\x1b[;H");
    printf("Controller Application\n");

	// Initialize LoRa Module
	if(!LoRa_Init()) {
		// Init failed
		Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
	}
	// Register onReceive callback
	onReceive(onReceiveCallback);

    // Intialize kill timer
	Cy_TCPWM_Counter_Init(killTimer_HW, killTimer_NUM, &killTimer_config);
	Cy_TCPWM_Counter_Enable(killTimer_HW, killTimer_NUM);
	Cy_TCPWM_SetInterruptMask(killTimer_HW, killTimer_NUM, CY_TCPWM_INT_ON_CC);
	Cy_SysInt_Init(&killTimer_interruptConfig, &killTimerCallback);
	NVIC_EnableIRQ(killTimer_interruptConfig.intrSrc);
	Cy_TCPWM_TriggerStart(killTimer_HW, killTimer_MASK);

    // SAR ADC Interrupt config struct
    cy_stc_sysint_t SAR_interruptConfig = {
    		.intrSrc = potADC_IRQ,
			.intrPriority = SAR_INTR_PRIORITY
    };

    // Initialize ADC
    Cy_SAR_Init(potADC_HW, &potADC_config);
    Cy_SAR_Enable(potADC_HW);
    Cy_SAR_ClearInterrupt(potADC_HW, CY_SAR_INTR_EOS);
    Cy_SAR_SetInterruptMask(potADC_HW, CY_SAR_INTR_EOS);
    Cy_SysInt_Init(&SAR_interruptConfig, SARCallback);
    NVIC_EnableIRQ(SAR_interruptConfig.intrSrc);

    // Intialize and start ADC timer
	Cy_TCPWM_Counter_Init(ADCTimer_HW, ADCTimer_NUM, &ADCTimer_config);
	Cy_TCPWM_Counter_Enable(ADCTimer_HW, ADCTimer_NUM);
	Cy_TCPWM_TriggerStart(ADCTimer_HW, ADCTimer_MASK);

	// Create the servo task
	xTaskCreate(ADC_task, (char *)"ADC_task", ADC_TASK_STACK_SIZE, 0, ADC_TASK_PRIORITY, &ADC_task_handle);

	// Start the scheduler
	vTaskStartScheduler();
	CY_ASSERT(0);
}

/* [] END OF FILE */
