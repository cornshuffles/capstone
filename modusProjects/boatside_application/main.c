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
 * Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company)
 *or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
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
#include "cy_retarget_io.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "boatProtocol.h"
#include <cy_DAC.h>
#include <cy_LoRa.h>
#include <cy_servo.h>

#define killTimer_INTR_PRIORITY (2U)

// FreeRTOS defines
#define THROTTLE_TASK_PRIORITY (2)
#define THROTTLE_TASK_STACK_SIZE (1024 * 1)
#define STEERING_TASK_PRIORITY (2)
#define STEERING_TASK_STACK_SIZE (1024 * 1)
#define SERVO_TASK_PRIORITY (2)
#define SERVO_TASK_STACK_SIZE (1024 * 1)

// FreeRTOS task handles
TaskHandle_t throttle_task_handle;
TaskHandle_t steering_task_handle;
TaskHandle_t servo_task_handle;

// Enable FreeRTOS debugging
volatile int uxTopUsedPriority;

// Steering, throttle and servo values
volatile uint8_t currentSteering = NONE;
volatile uint8_t currentThrottle = 0;
volatile uint8_t currentServo = 90;

cy_stc_sysint_t killTimer_interruptConfig = {
    .intrSrc = killTimer_IRQ, .intrPriority = killTimer_INTR_PRIORITY};

// Buffer to hold received data from LoRa in
volatile uint8_t rxBuffer[5] = "";

void killTimerCallback(void) {
  currentThrottle = 0;
  currentSteering = NONE;
  xTaskNotifyGive(throttle_task_handle);
  xTaskNotifyGive(steering_task_handle);
}

void onReceiveCallback(int packetLength) {
  // Read the message into the rxBuffer
  for (int i = 0; i < packetLength; i++) {
    rxBuffer[i] = read();
  }

  // Verify the password
  if (rxBuffer[4] == 78) {
    // Reset the kill timer
    Cy_TCPWM_Counter_SetCounter(killTimer_HW, killTimer_NUM, 0);

    // Parse the message
    switch (rxBuffer[0]) {
    case THROTTLE:
      if (rxBuffer[1] != currentThrottle) {
        currentThrottle = rxBuffer[1];
        xTaskNotifyGive(throttle_task_handle);
      }
      break;
    case STEERING:
      switch (rxBuffer[2]) {
      case LEFT:
        currentSteering = LEFT;
        xTaskNotifyGive(steering_task_handle);
        break;

      case RIGHT:
        currentSteering = RIGHT;
        xTaskNotifyGive(steering_task_handle);
        break;

      case NONE:
        currentSteering = NONE;
        break;
      }
      break;
    case SERVO:
      if (rxBuffer[3] != currentServo) {
        currentServo = rxBuffer[3];
        xTaskNotifyGive(servo_task_handle);
      }
      break;

    case KEEPALIVE:
      break;

    case KILL:
      currentThrottle = 0;
      currentSteering = NONE;
      xTaskNotifyGive(throttle_task_handle);
      xTaskNotifyGive(steering_task_handle);
      break;
    }
  }
}

void throttle_task(void *arg) {
  while (true) {
    // Wait for notification that a throttle command has been received then set
    // new throttle
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    DAC_Write((uint16_t)currentThrottle);
  }
}

void steering_task(void *arg) {
  while (true) {
    // Wait for notification that a throttle command has been received then set
    // new throttle
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    switch (currentSteering) {
    case LEFT:
      Cy_GPIO_Set(STEERING_LEFT_PORT, STEERING_LEFT_NUM);
      vTaskDelay(pdMS_TO_TICKS(500));
      Cy_GPIO_Clr(STEERING_LEFT_PORT, STEERING_LEFT_NUM);
      currentSteering = NONE;
      break;

    case RIGHT:
      Cy_GPIO_Set(STEERING_RIGHT_PORT, STEERING_RIGHT_NUM);
      vTaskDelay(pdMS_TO_TICKS(500));
      Cy_GPIO_Clr(STEERING_RIGHT_PORT, STEERING_RIGHT_NUM);
      currentSteering = NONE;
      break;

    case NONE:
      Cy_GPIO_Clr(STEERING_LEFT_PORT, STEERING_LEFT_NUM);
      Cy_GPIO_Clr(STEERING_RIGHT_PORT, STEERING_RIGHT_NUM);
      break;
    }
  }
}

void servo_task(void *arg) {
  while (true) {
    // Wait for notification that a throttle command has been received then set
    // new throttle
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    servo_Write((uint32_t)currentServo);
  }
}

int main(void) {
  cy_rslt_t result;

  /* Initialize the device and board peripherals */
  result = cybsp_init();
  if (result != CY_RSLT_SUCCESS) {
    CY_ASSERT(0);
  }

  /* Enable global interrupts */
  __enable_irq();

  // Enable Debug UART
  cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                      CY_RETARGET_IO_BAUDRATE);

  // Starting print
  printf("\x1b[2J\x1b[;H");
  printf("Boatside Application\n");

  // Initialize LoRa Module
  if (!LoRa_Init()) {
    // Init failed
    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
  }
  // Register onReceive callback
  onReceive(onReceiveCallback);

  // Initialize throttle
  DAC_Init();

  // Initialize servo
  servo_Init();

  // Intialize kill timer
  Cy_TCPWM_Counter_Init(killTimer_HW, killTimer_NUM, &killTimer_config);
  Cy_TCPWM_Counter_Enable(killTimer_HW, killTimer_NUM);
  Cy_TCPWM_SetInterruptMask(killTimer_HW, killTimer_NUM, CY_TCPWM_INT_ON_CC);
  Cy_SysInt_Init(&killTimer_interruptConfig, &killTimerCallback);
  NVIC_EnableIRQ(killTimer_interruptConfig.intrSrc);
  Cy_TCPWM_TriggerStart(killTimer_HW, killTimer_MASK);

  // Create the throttle task
  xTaskCreate(throttle_task, (char *)"throttle_task", THROTTLE_TASK_STACK_SIZE,
              0, THROTTLE_TASK_PRIORITY, &throttle_task_handle);
  // Create the steering task
  xTaskCreate(steering_task, (char *)"steering_task", STEERING_TASK_STACK_SIZE,
              0, STEERING_TASK_PRIORITY, &steering_task_handle);
  // Create the servo task
  xTaskCreate(servo_task, (char *)"servo_task", SERVO_TASK_STACK_SIZE, 0,
              SERVO_TASK_PRIORITY, &servo_task_handle);

  // Start the scheduler
  vTaskStartScheduler();
  CY_ASSERT(0);
}

/* [] END OF FILE */
