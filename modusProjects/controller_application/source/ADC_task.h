#ifndef ADC_TASK_H
	#define ADC_TASK_H

	// Macros
	#define ADC_TASK_PRIORITY   (2)
	#define ADC_TASK_STACK_SIZE (128 * 1)

// Function prototypes
void ADC_task(void *arg);

#endif

/* [] END OF FILE */
