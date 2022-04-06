// This is a partial port of https://github.com/sandeepmistry/arduino-LoRa intended to be used on Infineon's CY8CKIT-149 PSoC 4 kit

#include "cy_pdl.h"
#include "cybsp.h"

#include "cy_LoRa.h"

#define SPI_INTR_PRIORITY   (2U)

// LoRa struct to store state info in
struct LoRa myLoRa = {
		.interruptEnabled = false
};

// LoRa SPI context variable
cy_stc_scb_spi_context_t LoRa_spiContext;

// LoRa SPI interrupt configuration structure
const cy_stc_sysint_t spiIntrConfig = {
    .intrSrc      = LoRa_IRQ,
    .intrPriority = SPI_INTR_PRIORITY,
};

// LoRa SPI interrupt service routine
void SPI_Isr(void){
    Cy_SCB_SPI_Interrupt(LoRa_HW, &LoRa_spiContext);
}

// LoRa Module Functions

// Read/Write to Registers

// Function to perform a single read/write from/to the LoRa module.
// Params:	uint8_t address	- The address to read from or write to
//			uint8_t value	- The value to write, set this to 0 for reads
// Returns:	uint8_t - The value that was read
uint8_t singleTransfer(uint8_t address, uint8_t value){
	// Initialize tx buffer and rx buffer
	uint8_t txBuffer[2] = {address, value};
	uint8_t rxBuffer[2] = {0, 0};
	// Perform the transfer
	Cy_SCB_SPI_Transfer(LoRa_HW, txBuffer, rxBuffer, 2, &LoRa_spiContext);
	// Wait for transfer completion
	while(0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(LoRa_HW, &LoRa_spiContext))){}
	return rxBuffer[1];
}

// Function to perform a single read from the LoRa module.
// Params:	uint8_t address	- The address to read from
// Returns:	uint8_t - The value that was read
uint8_t readRegister(uint8_t address){
	return singleTransfer((address & 0x7f), 0);
}

// Function to perform a single write to the LoRa module.
// Params:	uint8_t address	- The address to write to
//			uint8_t			- The value to write
// Returns:	void
void writeRegister(uint8_t address, uint8_t value){
	singleTransfer((address | 0x80), value);
}

// Power Modes

// Function to place the LoRa module into the sleep power mode.
// Params:	void
// Returns:	void
void sleep(void){
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

// Function to place the LoRa module into the stand-by power mode.
// Params:	void
// Returns:	void
void stdby(void){
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

// Configuration/Utility

// Function to determine if a transmission is occurring
// Params:	void
// Returns:	bool - represents whether or not the LoRa module is transmitting. 1-transmitting, 0-not transmitting
bool isTransmitting(void){
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }
  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // Clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }
  return false;
}

// Function to set the LoRa module's gain
// Params:	uint8_t	- The gain. Valid range is 0-6
// Returns:	void
void setGain(uint8_t gain){
  // Check allowed range
  if (gain > 6) {
    gain = 6;
  }
  // Set to standby
  stdby();

  // set gain
  if (gain == 0) {
    // If gain = 0, enable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
  } else {
    // Disable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x00);
    // Clear Gain and set LNA boost
    writeRegister(REG_LNA, 0x03);
    // Set gain
    writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
  }
}

// Function to set the LoRa module's Over Current Protection level
// Params:	uint8_t	- The current level to trigger at in mA. Valid range is 45mA to 240mA
// Returns:	void
void setOCP(uint8_t mA){
  uint8_t ocpTrim = 27;
  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <= 240) {
    ocpTrim = (mA + 30) / 10;
  }
  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

// Function to set the LoRa module's Tx power
// Params:	uint32_t	- The power level in dBm. Valid range is 0-14 for PA_OUTPUT_RFO_PIN, 2-20 for PA_OUTPUT_PA_BOOST_PIN
//			uint32_t	- The output pin - PA_OUTPUT_RFO_PIN or PA_OUTPUT_PA_BOOST_PIN
// Returns:	void
void setTxPower(uint32_t level, uint32_t outputPin){
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }
    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }
      // Subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;
      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      // Default value PA_HF/LF or +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

// Function to set the LoRa module's frequency
// Params:	uint32_t	- frequency to set the module to in Hz. Valid range is 137MHz to 1020MHz
// Returns:	void
void setFrequency(uint32_t frequency){
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

// Function to put the LoRa module into explicit header mode
// Params:	void
// Returns:	void
void explicitHeaderMode(void){
	myLoRa.implicitHeaderMode = 0;
	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

// Function to put the LoRa module into implicit header mode
// Params:	void
// Returns:	void
void implicitHeaderMode(void){
	myLoRa.implicitHeaderMode = 1;
	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

// Function to initialize the LoRa module.
// Params:	void
// Returns:	uint8_t - A boolean representing whether or not the initialization succeeded. 1-success, 0-failure
uint8_t LoRa_Init(void){
	// Initialize SPI
	Cy_SCB_SPI_Init(LoRa_HW, &LoRa_config, &LoRa_spiContext);
	// Set active slave select line
	Cy_SCB_SPI_SetActiveSlaveSelect(LoRa_HW, CY_SCB_SPI_SLAVE_SELECT0);
	// Enable SPI Interrupt
	Cy_SysInt_Init(&spiIntrConfig, &SPI_Isr);
	NVIC_EnableIRQ(LoRa_IRQ);
	// Enable the SPI
	Cy_SCB_SPI_Enable(LoRa_HW);

	// Reset the LoRa module
	Cy_GPIO_Write(LoRa_Reset_PORT, LoRa_Reset_PIN, 0);
	Cy_SysLib_Delay(10);
	Cy_GPIO_Write(LoRa_Reset_PORT, LoRa_Reset_PIN, 1);
	Cy_SysLib_Delay(10);

	// Read from the LoRa module's version reg to verify that the read version is correct
	if(readRegister(REG_VERSION) != 0x12){
		return 0;
	}

	// Put the LoRa module into sleep mode
	sleep();

	// Set the LoRa module's frequency to 915MHz
	setFrequency(915000000);

	// Set base addresses
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

	// Set LNA boost
	writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

	// Set auto AGC
	writeRegister(REG_MODEM_CONFIG_3, 0x04);

	// Set output power to 20 dBm
	setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

	// Put in standby mode
	stdby();

	return 1;
}

// Function to begin building a packet for LoRa transfer
// Params:	bool implicitHeader	- true for implicit header mode, false for explicit header mode
// Returns:	uint8_t	- A boolean representing whether or not the function succeeded. 1-success, 0-failure
uint8_t beginPacket(bool implicitHeader){
  if (isTransmitting()) {
    return 0;
  }

  // Put in standby mode
  stdby();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // Reset FIFO address and payload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

// Function to perform a LoRa transfer
// Params:	bool async	- true for asynchronous, false for synchronous
// Returns:	uint8_t	- A boolean representing whether or not the function succeeded. 1-success, 0-failure
uint8_t endPacket(bool async){

  if ((async) && (myLoRa.onTxDone)){
      writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
  }

  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (!async) {
    // wait for TX done
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {}
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}

// Function to write to the packet to transfer via LoRa
// Params:	uint8_t *buffer	- The data to add to the packet
// Returns:	size_t	- The number of bytes written to the packet
size_t write(const uint8_t *buffer, size_t size){
  size_t currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

// ISR to handle DIO 0 interrupt
// Params:	void
// Returns:	void
void handleDio0Rise(void){

	// Get interrupt cause
	uint32_t intrSrc = Cy_GPIO_GetInterruptCause();
	// Check if the interrupt was from port 0
	if((0x00000001UL << LoRa_DIO_0_PORT_NUM) == (intrSrc & (0x00000001UL << LoRa_DIO_0_PORT_NUM))){
		// Clear the P0.1 interrupt
		Cy_GPIO_ClearInterrupt(LoRa_DIO_0_PORT, LoRa_DIO_0_NUM);
	}

	if(myLoRa.interruptEnabled){

  uint32_t irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

    if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
      // received a packet
      myLoRa.packetIndex = 0;

      // read packet length
      uint32_t packetLength = myLoRa.implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

      // set FIFO address to current RX address
      writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

      if (myLoRa.onReceive) {
        myLoRa.onReceive(packetLength);
      }
    }
    else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
      if (myLoRa.onTxDone) {
        myLoRa.onTxDone();
      }
    }
    }
  }
}

// Function to set up TxDone ISR
// Params:	void (*callback)()	- The callback function to execute
// Returns:	void
void onTxDone(void(*callback)(void)){
  myLoRa.onTxDone = callback;
	if (callback && !myLoRa.interruptEnabled) {
		static const cy_stc_sysint_t intrCfg =
		{
		/*.intrSrc =*/ LoRa_DIO_0_IRQ,
		/*.intrPriority =*/ 3UL
		};
		Cy_SysInt_Init(&intrCfg, &handleDio0Rise);
		Cy_GPIO_SetInterruptEdge(LoRa_DIO_0_PORT, LoRa_DIO_0_NUM, CY_GPIO_INTR_RISING);
		NVIC_EnableIRQ(intrCfg.intrSrc);
		myLoRa.interruptEnabled = true;
	}
}

// Function to put the radio into continuous receive mode
// Params:	size_t	- Only valid for implicit header mode, set this to 0 if using explicit header mode
// Returns:	void
void receive(size_t size){
  writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE
  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

// Function to set up recieve ISR
// Params:	void (*callback)()	- The callback function to execute
// Returns:	void
void onReceive(void(*callback)(int)){
  myLoRa.onReceive = callback;
	if (callback && !myLoRa.interruptEnabled) {
		static const cy_stc_sysint_t intrCfg =
		{
		/*.intrSrc =*/ LoRa_DIO_0_IRQ,
		/*.intrPriority =*/ 3UL
		};
		Cy_SysInt_Init(&intrCfg, &handleDio0Rise);
		Cy_GPIO_SetInterruptEdge(LoRa_DIO_0_PORT, LoRa_DIO_0_NUM, CY_GPIO_INTR_RISING);
		NVIC_EnableIRQ(intrCfg.intrSrc);
		myLoRa.interruptEnabled = true;
	}
}

// Function to check if a packet has been received
// Params:	size_t	- Only valid for implicit header mode, set this to 0 if using explicit header mode
// Returns:	size_t	- The size of the received packet in bytes. 0 if no packet was received
size_t parsePacket(size_t size){
  size_t packetLength = 0;
  uint32_t irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    myLoRa.packetIndex = 0;

    // read packet length
    if (myLoRa.implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    stdby();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

// Function to get the averaged RSSI of the last packet received in dBm
// Params:	void
// Returns:	uint32_t - The averaged RSSI of the last packet received in dBm
uint32_t packetRssi(void){
  return (readRegister(REG_PKT_RSSI_VALUE) - (myLoRa.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

// Function to get the estimated SNR of the last packet received in dB
// Params:	void
// Returns:	float - The estimated SNR of the last packet received in dB
float packetSnr(void){
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

// Function to get the current RSSI of the radio
// Params:	void
// Returns:	uint32_t - The current RSSI of the radio
uint32_t rssi(void){
  return (readRegister(REG_RSSI_VALUE) - (myLoRa.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

// Function to get the number of bytes available for reading
// Params:	void
// Returns:	uint8_t - The number of bytes available for reading
uint8_t available(void){
  return (readRegister(REG_RX_NB_BYTES) - myLoRa.packetIndex);
}

// Function to read a byte from the packet
// Params:	void
// Returns:	uint8_t - The byte read
uint8_t read(void){
  if (!available()) {
    return -1;
  }
  myLoRa.packetIndex++;
  return readRegister(REG_FIFO);
}

// Function to peek at the next byte from the packet
// Params:	void
// Returns:	uint8_t - The byte peeked at
uint8_t peek(){
  if (!available()) {
    return -1;
  }
  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);
  // read
  uint8_t b = readRegister(REG_FIFO);
  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}

// Function to enable CRC
// Params:	void
// Returns:	void
void enableCrc(void){
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

// Function to disable CRC
// Params:	void
// Returns:	void
void disableCrc(){
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

/* [] END OF FILE */
