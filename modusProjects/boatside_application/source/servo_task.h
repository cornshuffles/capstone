#ifndef SERVO_TASK_H
	#define SERVO_TASK_H

	// Macros
	#define SERVO_TASK_PRIORITY   (2)
	#define SERVO_TASK_STACK_SIZE (128 * 1)

// Function prototypes
void servo_task(void *arg);

#endif

/* [] END OF FILE */
