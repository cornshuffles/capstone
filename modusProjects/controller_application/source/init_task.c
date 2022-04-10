#include "cy_pdl.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ADC_task.h"
#include "LoRa_task.h"

#include "boatProtocol.h"
#include <cy_LoRa.h>

#define SAR_INTR_PRIORITY       (3U)
#define killTimer_INTR_PRIORITY (2U)
#define left_btn_INTR_PRIORITY  (2U)
#define right_btn_INTR_PRIORITY (2U)
#define kill_btn_INTR_PRIORITY  (2U)

extern TaskHandle_t ADC_task_handle;
extern TaskHandle_t LoRa_task_handle;

extern QueueHandle_t LoRa_queue_handle;

volatile uint8_t killEnabled = 0;

// SAR ADC Interrupt config struct
cy_stc_sysint_t SAR_interruptConfig = {.intrSrc = potADC_IRQ, .intrPriority = SAR_INTR_PRIORITY};

// killTimer interrupt config struct
cy_stc_sysint_t killTimer_interruptConfig = {.intrSrc = killTimer_IRQ, .intrPriority = killTimer_INTR_PRIORITY};

// btn interrupt config structure
cy_stc_sysint_t btn_interruptConfig = {.intrSrc = left_btn_IRQ, .intrPriority = left_btn_INTR_PRIORITY};

// ISR that runs when the ADC finishes a scan. The ADC's scans are triggered by a timer block.
// Params:	void
// Returns:	void
void SARCallback(void) {
	Cy_SAR_ClearInterrupt(potADC_HW, CY_SAR_INTR_EOS);
	vTaskNotifyGiveFromISR(ADC_task_handle, NULL);
}

// ISR that runs when the killtimer reaches 3 seconds.
// Params:	void
// Returns:	void
void killTimerCallback(void) {
	Cy_TCPWM_ClearInterrupt(killTimer_HW, killTimer_NUM, CY_TCPWM_INT_ON_CC);
	// Send keepalive command to LoRa task
	uint8_t valueToSend = (uint8_t)KEEPALIVE;
	xQueueSendToBackFromISR(LoRa_queue_handle, &valueToSend, NULL);
}

// ISR that runs when any of the buttons are pushed. Ports 4 and 5 on the PSoC 4100s share an IRQ.
// Params:	void
// Returns:	void
void btn_Callback(void) {
	// Get interrupt cause
	if(Cy_GPIO_GetInterruptStatus(left_btn_PORT, left_btn_NUM)) {
		// Clear the interrupt and send steering command to LoRa task
		Cy_GPIO_ClearInterrupt(left_btn_PORT, left_btn_NUM);
		uint8_t valueToSend = (uint8_t)LEFT;
		xQueueSendToBackFromISR(LoRa_queue_handle, &valueToSend, NULL);
	} else if(Cy_GPIO_GetInterruptStatus(right_btn_PORT, right_btn_NUM)) {
		// Clear the interrupt and send steering command to LoRa task
		Cy_GPIO_ClearInterrupt(right_btn_PORT, right_btn_NUM);
		uint8_t valueToSend = (uint8_t)RIGHT;
		xQueueSendToBackFromISR(LoRa_queue_handle, &valueToSend, NULL);
	} else if(Cy_GPIO_GetInterruptStatus(kill_btn_PORT, kill_btn_NUM)) {
		// Clear the interrupt and send queue message to LoRa task
		Cy_GPIO_ClearInterrupt(kill_btn_PORT, kill_btn_NUM);
		killEnabled ^= 1;    // Toggle the global kill state
		if(killEnabled) {
			uint8_t valueToSend = (uint8_t)KILL;
			xQueueSendToBackFromISR(LoRa_queue_handle, &valueToSend, NULL);
		}
	}
}

// Task to initialize hardware and start the other tasks.
// Params:	void
// Returns:	void
void init_task(void *arg) {
	// Initialize GPIO interrupts
	// On the PSoC 4100s, ports 4 and 5 share an IRQ. See: https://tinyurl.com/2524em2z
	Cy_SysInt_Init(&btn_interruptConfig, &btn_Callback);
	// Send the button through the glitch filter
	Cy_GPIO_SetFilter(left_btn_PORT, left_btn_NUM);
	// Falling edge interrupt
	Cy_GPIO_SetInterruptEdge(left_btn_PORT, left_btn_NUM, CY_GPIO_INTR_FALLING);
	/* Enable the interrupt */
	NVIC_EnableIRQ(btn_interruptConfig.intrSrc);
	// right_btn
	// Falling edge interrupt
	Cy_GPIO_SetInterruptEdge(right_btn_PORT, right_btn_NUM, CY_GPIO_INTR_FALLING);
	// kill_btn
	Cy_GPIO_SetFilter(kill_btn_PORT, kill_btn_NUM);
	// Falling edge interrupt
	Cy_GPIO_SetInterruptEdge(kill_btn_PORT, kill_btn_NUM, CY_GPIO_INTR_FALLING);

	// Initialize LoRa Module
	LoRa_Init();

	// Intialize kill timer
	Cy_TCPWM_Counter_Init(killTimer_HW, killTimer_NUM, &killTimer_config);
	Cy_TCPWM_Counter_Enable(killTimer_HW, killTimer_NUM);
	Cy_TCPWM_SetInterruptMask(killTimer_HW, killTimer_NUM, CY_TCPWM_INT_ON_CC);
	Cy_SysInt_Init(&killTimer_interruptConfig, &killTimerCallback);
	NVIC_EnableIRQ(killTimer_interruptConfig.intrSrc);
	Cy_TCPWM_TriggerStart(killTimer_HW, killTimer_MASK);

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

	// Create the ADC and LoRa tasks
	xTaskCreate(ADC_task, (char *)"ADC_task", ADC_TASK_STACK_SIZE, 0, ADC_TASK_PRIORITY, &ADC_task_handle);
	xTaskCreate(LoRa_task, (char *)"LoRa_task", LORA_TASK_STACK_SIZE, 0, LORA_TASK_PRIORITY, &LoRa_task_handle);

	while(true) {
		vTaskDelay(1);
	}
}

/* [] END OF FILE */
