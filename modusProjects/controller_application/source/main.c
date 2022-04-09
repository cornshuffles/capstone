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
#include "queue.h"

#include "init_task.h"

#include "boatProtocol.h"
#include <cy_LoRa.h>

/* ==========Global Vars========== */
// Enable FreeRTOS debugging
volatile int uxTopUsedPriority = configMAX_PRIORITIES - 1;

// FreeRTOS task handles
TaskHandle_t ADC_task_handle;
TaskHandle_t LoRa_task_handle;
TaskHandle_t init_task_handle;

// FreeRTOS queue handle
QueueHandle_t LoRa_queue_handle = 0;

// Throttle and servo values
volatile uint8_t currentThrottle = 0;
volatile uint8_t currentServoAngle = 0;

// Main of the application, initializes HW, FreeRTOS tasks, and starts the scheduler
// Params:	void
// Returns:	int
int main(void) {
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();
	if(result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	// Enable Debug UART
	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

	// Starting print
	//printf("\x1b[2J\x1b[;H");
	//printf("Controller Application\n");

	// Create the FreeRTOS queue
	LoRa_queue_handle = xQueueCreate(10, sizeof(uint8_t));

	// Create the servo task
	xTaskCreate(init_task, (char *)"servo_task", INIT_TASK_STACK_SIZE, 0, INIT_TASK_PRIORITY, &init_task_handle);

	// Start the scheduler
	vTaskStartScheduler();
	CY_ASSERT(0);
}

/* [] END OF FILE */
