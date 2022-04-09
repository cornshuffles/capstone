#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "steering_task.h"
#include "boatProtocol.h"

extern volatile uint8_t currentSteering;

// Task to drive the steering pins.
// Params:	void
// Returns:	void
void steering_task(void *arg) {
	while(true) {
		// Wait for notification that a throttle command has been received then set new throttle
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		switch(currentSteering) {
			case LEFT:
				Cy_GPIO_Set(steering_left_PORT, steering_left_NUM);
				vTaskDelay(pdMS_TO_TICKS(500));
				Cy_GPIO_Clr(steering_left_PORT, steering_left_NUM);
				currentSteering = NONE;
				break;

			case RIGHT:
				Cy_GPIO_Set(steering_right_PORT, steering_right_NUM);
				vTaskDelay(pdMS_TO_TICKS(500));
				Cy_GPIO_Clr(steering_right_PORT, steering_right_NUM);
				currentSteering = NONE;
				break;

			case NONE:
				Cy_GPIO_Clr(steering_left_PORT, steering_left_NUM);
				Cy_GPIO_Clr(steering_right_PORT, steering_right_NUM);
				break;
		}
	}
}

/* [] END OF FILE */
