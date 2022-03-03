/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC4  Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define BTN_INTR_PRIORITY	(3UL)
#define SPI_INTR_PRIORITY   (3UL)
#define PORT_INTR_MASK  (0x00000001UL << CYBSP_USER_BTN_PORT_NUM)

// Flag gets set in GPIO interrupt
volatile uint32_t flag = 0;

// BTN isr
void Port3_Isr(void){
	// Check interrupt cause
	uint32_t intrSrc = Cy_GPIO_GetInterruptCause();
	if(PORT_INTR_MASK == (intrSrc & PORT_INTR_MASK)){
		// Clear the P3.7 interrupt
		Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
		// Toggle led and set flag
		Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
		flag = 1;
	}
}

// BTN interrupt configuration structure
cy_stc_sysint_t BTN_intrCfg =
{
	.intrSrc = CYBSP_USER_BTN_IRQ,
	.intrPriority = BTN_INTR_PRIORITY
};

// DAC SPI Context variable
cy_stc_scb_spi_context_t DAC_SpiContext;

// DAC SPI Isr
void SPI_Isr(void){
    Cy_SCB_SPI_Interrupt(DAC_HW, &DAC_SpiContext);
}

// SPI interrupt configuration structure
const cy_stc_sysint_t SPI_IntrCfg = {
    .intrSrc      = DAC_IRQ,
    .intrPriority = SPI_INTR_PRIORITY,
};

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    // Enable Debug printing
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    // Enable BTN Interrupt
    Cy_SysInt_Init(&BTN_intrCfg, &Port3_Isr);
    // Send the button through the glitch filter
	Cy_GPIO_SetFilter(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
	// Falling edge interrupt
	Cy_GPIO_SetInterruptEdge(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM, CY_GPIO_INTR_FALLING);
	// Enable the interrupt
	NVIC_EnableIRQ(BTN_intrCfg.intrSrc);

	// Init and Enable SPI SCB
	Cy_SCB_SPI_Init(DAC_HW, &DAC_config, &DAC_SpiContext);
	Cy_SCB_SPI_SetActiveSlaveSelect(DAC_HW, CY_SCB_SPI_SLAVE_SELECT0);
	// Enable SPI Interrupt
	Cy_SysInt_Init(&SPI_IntrCfg, &SPI_Isr);
	NVIC_EnableIRQ(DAC_IRQ);
	Cy_SCB_SPI_Enable(DAC_HW);

	uint16_t transmit_datum = 0x4000;

    for (;;){
    	if(flag){
			Cy_SCB_SPI_Transfer(DAC_HW, &transmit_datum, NULL, 1U, &DAC_SpiContext);
			printf("Data sent\n");
			flag = 0;
    	}
    }
}

/* [] END OF FILE */
