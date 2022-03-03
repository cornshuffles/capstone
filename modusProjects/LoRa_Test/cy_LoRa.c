#include "cy_pdl.h"
#include "cybsp.h"

#include "cy_LoRa.h"

#define SPI_INTR_PRIORITY   (3U)

cy_stc_scb_spi_context_t LoRa_spiContext;

/* Populate configuration structure */
const cy_stc_sysint_t spiIntrConfig = {
    .intrSrc      = LoRa_IRQ,
    .intrPriority = SPI_INTR_PRIORITY,
};

void SPI_Isr(void){
    Cy_SCB_SPI_Interrupt(LoRa_HW, &LoRa_spiContext);
}

uint8_t LoRa_Init(){
	// Initialize SPI
	Cy_SCB_SPI_Init(LoRa_HW, &LoRa_config, &LoRa_spiContext);
	// Set active slave select line
	Cy_SCB_SPI_SetActiveSlaveSelect(LoRa_HW, CY_SCB_SPI_SLAVE_SELECT0);
	// Enable SPI Interrupt
	Cy_SysInt_Init(&spiIntrConfig, &SPI_Isr);
	NVIC_EnableIRQ(LoRa_IRQ);
	// Enable the SPI
	Cy_SCB_SPI_Enable(LoRa_HW);
	// Read from version reg to make sure everything is working
	uint8_t txBuffer[2];
	txBuffer[1] = 0;
	txBuffer[0] = REG_VERSION & 0x7f;
	uint8_t rxBuffer[2];
	cy_en_scb_spi_status_t status;

	Cy_GPIO_Write(LoRa_Reset_PORT, LoRa_Reset_PIN, 0);
	Cy_SysLib_Delay(10);
	Cy_GPIO_Write(LoRa_Reset_PORT, LoRa_Reset_PIN, 1);
	Cy_SysLib_Delay(10);

	// Perform the read
	status = Cy_SCB_SPI_Transfer(LoRa_HW, txBuffer, rxBuffer, 2, &LoRa_spiContext);
	// Wait for transfer completion
	while(0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(LoRa_HW, &LoRa_spiContext))){}
	// Verify the version
	printf("Status: %d", status);
	printf("rxBuffer[0] = %x\n", rxBuffer[0]);
	return rxBuffer[0] == 0x12;
}


//uint8_t singleTransfer(uint8_t address, uint8_t value){
//
//}
