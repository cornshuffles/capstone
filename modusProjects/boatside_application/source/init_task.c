#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "throttle_task.h"
#include "steering_task.h"
#include "servo_task.h"

#include "boatProtocol.h"
#include <cy_LoRa.h>
#include <cy_DAC.h>
#include <cy_servo.h>

#define killTimer_INTR_PRIORITY (2U)

// Buffer to hold received data from LoRa in
volatile uint8_t rxBuffer[5];

extern TaskHandle_t throttle_task_handle;
extern TaskHandle_t steering_task_handle;
extern TaskHandle_t servo_task_handle;

extern volatile uint8_t currentSteering;
extern volatile uint8_t currentThrottle;
extern volatile uint8_t currentServo;

// killTimer interrupt config struct
cy_stc_sysint_t killTimer_interruptConfig = {.intrSrc = killTimer_IRQ, .intrPriority = killTimer_INTR_PRIORITY};

// Interrupt service routine to handle the kill timer reaching 5 seconds
// Params:	void
// Returns:	void
void killTimerCallback(void) {
	Cy_TCPWM_ClearInterrupt(killTimer_HW, killTimer_NUM, CY_TCPWM_INT_ON_CC);
	currentThrottle = 0;
	currentSteering = NONE;
	vTaskNotifyGiveFromISR(throttle_task_handle, NULL);
	vTaskNotifyGiveFromISR(steering_task_handle, NULL);
}

// Interrupt service routine to handle receiving a message via LoRa.
// Params:	void
// Returns:	void
void onReceiveCallback(int packetLength) {
	// Read the message into the rxBuffer
	for(int i = 0; i < packetLength; i++) {
		rxBuffer[i] = read();
	}

	// Verify the password
	if(rxBuffer[4] == 78) {
		// Reset the kill timer
		Cy_TCPWM_Counter_SetCounter(killTimer_HW, killTimer_NUM, 0);

		Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);

		// Parse the message
		switch(rxBuffer[0]) {
			case THROTTLE:
				if(rxBuffer[1] != currentThrottle) {
					currentThrottle = rxBuffer[1];
					vTaskNotifyGiveFromISR(throttle_task_handle, NULL);
				}
				break;
			case STEERING:
				switch(rxBuffer[2]) {
					case LEFT:
						currentSteering = LEFT;
						vTaskNotifyGiveFromISR(steering_task_handle, NULL);
						break;

					case RIGHT:
						currentSteering = RIGHT;
						vTaskNotifyGiveFromISR(steering_task_handle, NULL);
						break;

					case NONE: currentSteering = NONE; break;
				}
				break;
			case SERVO:
				if(rxBuffer[3] != currentServo) {
					currentServo = rxBuffer[3];
					vTaskNotifyGiveFromISR(servo_task_handle, NULL);
				}
				break;

			case KEEPALIVE: break;

			case KILL:
				currentThrottle = 0;
				currentSteering = NONE;
				vTaskNotifyGiveFromISR(throttle_task_handle, NULL);
				vTaskNotifyGiveFromISR(steering_task_handle, NULL);
				break;
		}
	}
}

// Task to initialize hardware and start the other tasks.
// Params:	void
// Returns:	void
void init_task(void *arg) {
	// Initialize LoRa Module
	LoRa_Init();

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
	xTaskCreate(throttle_task, (char *)"throttle_task", THROTTLE_TASK_STACK_SIZE, 0, THROTTLE_TASK_PRIORITY, &throttle_task_handle);
	// Create the steering task
	xTaskCreate(steering_task, (char *)"steering_task", STEERING_TASK_STACK_SIZE, 0, STEERING_TASK_PRIORITY, &steering_task_handle);
	// Create the servo task
	xTaskCreate(servo_task, (char *)"servo_task", SERVO_TASK_STACK_SIZE, 0, SERVO_TASK_PRIORITY, &servo_task_handle);

	// Place the radio into receive mode
	receive(0);

	while(true) {
		vTaskDelay(1);
	}
}

/* [] END OF FILE */
