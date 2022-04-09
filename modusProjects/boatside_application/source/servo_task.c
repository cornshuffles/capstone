#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "servo_task.h"
#include "boatProtocol.h"
#include <cy_servo.h>

extern volatile uint8_t currentServo;

// Task to drive the servo.
// Params:	void
// Returns:	void
void servo_task(void *arg) {
	while(true) {
		// Wait for notification that a throttle command has been received then set new throttle
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		servo_Write((uint32_t)currentServo);
	}
}

/* [] END OF FILE */
