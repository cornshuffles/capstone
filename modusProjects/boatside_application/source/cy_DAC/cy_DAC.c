#include "cy_pdl.h"
#include "cybsp.h"

#include "cy_DAC.h"

#define DAC_SPI_INTR_PRIORITY (2U)

// DAC SPI Context variable
cy_stc_scb_spi_context_t DAC_SpiContext;

// SPI interrupt configuration structure
const cy_stc_sysint_t SPI_IntrCfg = {
.intrSrc = DAC_IRQ,
.intrPriority = DAC_SPI_INTR_PRIORITY,
};

// DAC SPI Isr
void DAC_SPI_Isr(void) { Cy_SCB_SPI_Interrupt(DAC_HW, &DAC_SpiContext); }

void DAC_Init(void) {
	// Init and Enable SPI SCB
	Cy_SCB_SPI_Init(DAC_HW, &DAC_config, &DAC_SpiContext);
	Cy_SCB_SPI_SetActiveSlaveSelect(DAC_HW, CY_SCB_SPI_SLAVE_SELECT0);
	// Enable SPI Interrupt
	Cy_SysInt_Init(&SPI_IntrCfg, &DAC_SPI_Isr);
	NVIC_EnableIRQ(DAC_IRQ);
	Cy_SCB_SPI_Enable(DAC_HW);
	DAC_Write(0x0000);
}

void DAC_Write(uint16_t data) { Cy_SCB_SPI_Transfer(DAC_HW, &data, NULL, 1U, &DAC_SpiContext); }

/* [] END OF FILE */
