#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "LoRa_task.h"
#include "boatProtocol.h"
#include <cy_LoRa.h>
#include <stdio.h>

extern QueueHandle_t LoRa_queue_handle;
extern volatile uint8_t currentThrottle;
extern volatile uint8_t currentServoAngle;
extern volatile uint8_t killEnabled;

// Task to handle sending messages over the LoRa radio
// Params:	void
// Returns:	int
void LoRa_task(void *arg) {
	uint8_t rxBuffer;       // Buffer to receive items from the queue into
	uint8_t txBuffer[5];    // Buffer to send over LoRa
	txBuffer[4] = 78;       // "Password"
	size_t txSize = sizeof(txBuffer);
	size_t bytesWritten = 0;

	while(true) {
		// Wait for command to be sent in the queue
		xQueueReceive(LoRa_queue_handle, &rxBuffer, portMAX_DELAY);
		switch(rxBuffer) {
			case THROTTLE:
				txBuffer[0] = (uint8_t)THROTTLE;
				txBuffer[1] = currentThrottle;
				txBuffer[2] = 0;
				txBuffer[3] = 0;
				//printf("THROTTLE: %d\n", currentThrottle);
				break;
			case LEFT:
				txBuffer[0] = (uint8_t)STEERING;
				txBuffer[1] = 0;
				txBuffer[2] = (uint8_t)LEFT;
				txBuffer[3] = 0;
				//printf("STEERING: %d\n", LEFT);
				break;
			case RIGHT:
				txBuffer[0] = (uint8_t)STEERING;
				txBuffer[1] = 0;
				txBuffer[2] = (uint8_t)RIGHT;
				txBuffer[3] = 0;
				//printf("STEERING: %d\n", RIGHT);
				break;
			case SERVO:
				txBuffer[0] = (uint8_t)SERVO;
				txBuffer[1] = 0;
				txBuffer[2] = 0;
				txBuffer[3] = currentServoAngle;
				//printf("SERVO: %d\n", currentServoAngle);
				break;
			case KEEPALIVE:
				txBuffer[0] = (uint8_t)KEEPALIVE;
				txBuffer[1] = 0;
				txBuffer[2] = 0;
				txBuffer[3] = 0;
				//printf("KEEPALIVE \n");
				break;
			case KILL:
				txBuffer[0] = (uint8_t)KILL;
				txBuffer[1] = 0;
				txBuffer[2] = 0;
				txBuffer[3] = 0;
				//printf("KILL \n");
				Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
				// Do the transmission
				// Begin packet
				if(!beginPacket(false)) {
					// beginPacket failed
					Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
				}

				// Write to packet
				bytesWritten = write(txBuffer, txSize);
				if(bytesWritten != txSize) {
					// write failed
					Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
				}

				// End packet and transmit
				if(!endPacket(false)) {
					// endPacket failed
					Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
				}

				// Reset the kill timer
				Cy_TCPWM_Counter_SetCounter(killTimer_HW, killTimer_NUM, 0);
				break;
		}

		if(!killEnabled) {
			//printf("SENT\n");
			Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
			// Do the transmission
			// Begin packet
			if(!beginPacket(false)) {
				// beginPacket failed
				Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
			}

			// Write to packet
			bytesWritten = write(txBuffer, txSize);
			if(bytesWritten != txSize) {
				// write failed
				Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
			}

			// End packet and transmit
			if(!endPacket(false)) {
				// endPacket failed
				Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 1);
			}
		}
		// Reset the kill timer
		Cy_TCPWM_Counter_SetCounter(killTimer_HW, killTimer_NUM, 0);
	}
}

/* [] END OF FILE */
