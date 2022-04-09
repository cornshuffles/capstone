#include "cy_pdl.h"
#include "cybsp.h"

#include "cy_servo.h"

void servo_Init(void) {
	Cy_TCPWM_PWM_Init(servo_HW, servo_NUM, &servo_config);
	Cy_TCPWM_PWM_Enable(servo_HW, servo_NUM);
	Cy_TCPWM_TriggerReloadOrIndex(servo_HW, servo_MASK);
}

void servo_Write(uint32_t angle) {
	// 0 degrees - 1200
	// 90 degrees - 720
	// 180 degrees - 350
	Cy_TCPWM_PWM_SetCompare0(servo_HW, servo_NUM, (((angle * 850) / 180) + 350));
}

/* [] END OF FILE */
