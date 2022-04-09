#ifndef THROTTLE_TASK_H
	#define THROTTLE_TASK_H

	// Macros
	#define THROTTLE_TASK_PRIORITY   (2)
	#define THROTTLE_TASK_STACK_SIZE (128 * 1)

// Function prototypes
void throttle_task(void *arg);

#endif

/* [] END OF FILE */
