#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "throttle_task.h"
#include "boatProtocol.h"
#include <cy_DAC.h>
#include <cy_LoRa.h>

extern volatile uint8_t currentThrottle;

// Task to drive the DAC.
// Params:	void
// Returns:	void
void throttle_task(void *arg) {
	while(true) {
		// Wait for notification that a throttle command has been received then set new throttle
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		uint16_t throttleToWrite = (currentThrottle * 0xFFFF) / 0xFF;
		DAC_Write(throttleToWrite);
	}
}

/* [] END OF FILE */
