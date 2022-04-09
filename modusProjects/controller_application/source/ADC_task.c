#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ADC_task.h"
#include "boatProtocol.h"
#include <stdlib.h>

extern QueueHandle_t LoRa_queue_handle;
extern volatile uint8_t currentThrottle;
extern volatile uint8_t currentServoAngle;

void ADC_task(void *arg) {
	uint8_t valueToSend;

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
		if(abs((int16_t)throttle - (int16_t)currentThrottle) > 3) {
			currentThrottle = throttle;
			valueToSend = (uint8_t)THROTTLE;
			xQueueSendToBack(LoRa_queue_handle, &valueToSend, 0);
		}
		if(abs((int16_t)servoAngle - (int16_t)currentServoAngle) > 3) {
			currentServoAngle = servoAngle;
			valueToSend = (uint8_t)SERVO;
			xQueueSendToBack(LoRa_queue_handle, &valueToSend, 0);
		}
	}
}

/* [] END OF FILE */
