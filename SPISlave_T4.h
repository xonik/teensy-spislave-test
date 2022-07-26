#if !defined(_SPISlave_T4_H_)
#define _SPISlave_T4_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include <SPI.h>

typedef enum SPI_BITS {
  SPI_8_BITS = 8,
  SPI_16_BITS = 16,
  SPI_32_BITS = 32,
} SPI_BITS;

typedef void (*_SPI_ptr)();

#define SPISlave_T4_CLASS template<SPIClass* port = nullptr, SPI_BITS bits = SPI_8_BITS>
#define SPISlave_T4_FUNC template<SPIClass* port, SPI_BITS bits>
#define SPISlave_T4_OPT SPISlave_T4<port, bits>

// 0x40394000 is LPSPI1 memory map base address (48.5.1.1 in manual, p2867)
// - Offset is in 8bit blocks whereas number here is in 32 bit blocks
#define SLAVE_PARAM spiAddr[1] // 4 = 0x04, Parameter (PARAM)
#define SLAVE_CR spiAddr[4] // 16 = 0x10, Control (CR)
#define SLAVE_FCR spiAddr[22] //88 = 0x58, FIFO Control
#define SLAVE_IER spiAddr[6] // 24 = 0x18, Interrupt Enable
#define SLAVE_CFGR0 spiAddr[8] // 32 = 0x20, Configuration 0
#define SLAVE_CFGR1 spiAddr[9] // 36 = 0x24, Configuration 1
#define SLAVE_TCR spiAddr[24] // 96 = 0x60 
//#define SLAVE_TDR spiAddr[25] // 100 = 0x64, Transmit Data
#define SLAVE_TDR(x) spiAddr[25] = (x)
#define SLAVE_RDR spiAddr[29] // 116 = 0x74, Receive Data
#define SLAVE_SR spiAddr[5] // 20 = 0x14, Status (SR) 
#define SLAVE_FSR spiAddr[23] // 92 = 0x5C, FIFO Status

// Actually a command, setting TCR
#define SLAVE_TCR_REFRESH spiAddr[24] = (0UL << 27) | LPSPI_TCR_FRAMESZ(8 - 1)

// Interrupts:
// TDF/TDIE: Data can be written to transmit FIFO
// RDF/RDIE: Data can be read from receive FIFO
// WCF/WCIE: Word complete: When enough bytes have been transferred to fill a word
// FCF/FCIE: Frame complete - When PCS goes high.
// TCF/TCIE: Transfer Complete - Used for send. When all data in transmit/command FIFO is empty.
// TEF/TEIE: Transmit/command FIFO underrun
// REF/REIE: Receive FIFO overflow
// DMF/DMIE: Data has matched configured data match value

// MBF: Not an interrupt: Module busy status flag

extern SPIClass SPI;

class SPISlave_T4_Base {
  public:
    virtual void SLAVE_ISR();
};

int32_t received[256];
int32_t receivedSR[256];
int32_t receivedMillis[256];
uint8_t receivedTxfifo[256];
uint8_t receivedRxfifo[256];
uint8_t recindex = 0;
boolean dataready = false;
uint8_t receives = 0;
uint8_t lastPrint = 0;
uint8_t outs = 0;
uint8_t in = 0;

static SPISlave_T4_Base* _LPSPI4 = nullptr;

SPISlave_T4_CLASS class SPISlave_T4 : public SPISlave_T4_Base {
  public:
    SPISlave_T4();
    void begin();
    uint32_t transmitErrors();
    void printSr();
    void srStatus(uint32_t sr);
    void printReceived();
    void fillTx();

  private:
    volatile uint32_t *spiAddr;
    _SPI_ptr _spihandler = nullptr;
    void SLAVE_ISR();
    uint32_t nvic_irq = 0;    
};

void lpspi4_slave_isr() {
  _LPSPI4->SLAVE_ISR();
}

// Constructor
SPISlave_T4_FUNC SPISlave_T4_OPT::SPISlave_T4() {
  if ( port == &SPI ) {
    _LPSPI4 = this;
    uint8_t _portnum = 3;
    spiAddr = &(*(volatile uint32_t*)(0x40394000 + (0x4000 * _portnum)));

    // Without this, nothing is received.
    // CCM Clock Gating Register 1 - CG3 - LPSPI4 Clocks enable
    CCM_CCGR1 |= (3UL << 6);

    // Interrupt vector for LPSPI
    nvic_irq = 32 + _portnum;

    // Interrupt Service Routine
    _VectorsRam[16 + nvic_irq] = lpspi4_slave_isr;

    volatile uint32_t *spireg = &(*(volatile uint32_t*)(0x401F84EC + (_portnum * 0x10)));
    spireg[0] = 0; /* PCS0_SELECT_INPUT */
    spireg[1] = 0; /* SCK_SELECT_INPUT */
    spireg[2] = 0; /* SDI_SELECT_INPUT */
    spireg[3] = 0; /* SDO_SELECT_INPUT */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
    
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
    
  } 
}

// Definitions
// Word: 0-padded 32 bit.
// Frame: A number of bits in a total transfer, may be more than
//        1 word, in which case frame is split into multiple words.
//        As long as PCS is asserted (low). Divisible by 2


SPISlave_T4_FUNC void SPISlave_T4_OPT::printSr() {
  Serial.print("Last: ");Serial.print(lastPrint);Serial.print(", receives ");Serial.println(receives);
  srStatus(SLAVE_SR);
}
SPISlave_T4_FUNC void SPISlave_T4_OPT::srStatus(uint32_t sr) {
  if(sr & LPSPI_SR_MBF) Serial.print("Module busy, "); // Not an interrupt
  if(sr & LPSPI_SR_DMF) Serial.print("Data matched, ");
  if(sr & LPSPI_SR_REF) Serial.print("RX FIFO overflow, ");
  if(sr & LPSPI_SR_TEF) Serial.print("TX FIFO underrun, ");
  if(sr & LPSPI_SR_TCF) Serial.print("All TX completed, ");
  if(sr & LPSPI_SR_FCF) Serial.print("Frame completed, ");
  if(sr & LPSPI_SR_WCF) Serial.print("Word completed, ");
  if(sr & LPSPI_SR_RDF) Serial.print("RX data ready, ");
  if(sr & LPSPI_SR_TDF) Serial.print("TX data ready, ");
}

uint32_t repeat = 0;

SPISlave_T4_FUNC void SPISlave_T4_OPT::printReceived() {
  if(lastPrint != receives) {
    for(uint8_t i = lastPrint; i<receives; i++) {
      //Serial.print("\nSR ");Serial.print(i);Serial.print(" ");Serial.println(receivedSR[i]);
      Serial.print("\ni ");Serial.print(i);Serial.print(", Last ");Serial.print(lastPrint);Serial.print(", receives ");Serial.println(receives);
      Serial.print(receivedMillis[i]);
      Serial.print(": ");
      srStatus(receivedSR[i]);
      //Serial.print("\nTXFIFO: ");Serial.print(receivedTxfifo[i]);Serial.print(", RXFIFO: ");Serial.print(receivedRxfifo[i]);
     // Serial.print(" Received: ");Serial.println(received[i]);
    }
    
    lastPrint = receives;  
  }
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::fillTx() {
  Serial.print("Fill, ");Serial.print("Len ");Serial.println(SLAVE_FSR & 0xFF);
  if((SLAVE_FSR & 0xFF) < 4) {
    SLAVE_TDR(1);
    SLAVE_TDR(2);
    SLAVE_TDR(3);
    SLAVE_TDR(4);
    SLAVE_TDR(5);
    SLAVE_TDR(6);
    SLAVE_TDR(7);
    SLAVE_TDR(8);
  }
}

uint32_t prev = 0;

SPISlave_T4_FUNC void SPISlave_T4_OPT::SLAVE_ISR() {

  uint32_t data = 0;
  // If data received
  if(SLAVE_SR & 0x2) {
    data = SLAVE_RDR;
    uint32_t next = millis();
    Serial.print(in++);Serial.print(" ");Serial.print(next-prev);Serial.print(" ");Serial.println(data);
    prev = next;
  }
  SLAVE_SR = 0x3F00;
  
  
  if(data){
    data = 0;
  }
  
  asm volatile ("dsb");
}

SPISlave_T4_FUNC void SPISlave_T4_OPT::begin() {
  // 2, Software reset Reset SPI Module
  SLAVE_CR = LPSPI_CR_RST;

  // Disable everything but especially 0: MEN - Module enable
  SLAVE_CR = 0;

  // If FCR is 10001, SR = 1, if FCR = 0, SR = 2 when entering ISR
  //SLAVE_FCR = 0x00010001; // if set to 0x10001: 1x watermark for RX and TX 
  SLAVE_FCR = 0;
  
  // Enabled interrupts
  SLAVE_IER = 0x3F00;
  //SLAVE_IER = LPSPI_IER_FCIE; // Frame complete
  //SLAVE_IER = LPSPI_IER_WCIE; // Word complete

  // 9: RDMO: Received data is stored in receive FIFO as i normal operations
  // 8: CIRCFIFO: Circular FIFO is disabled
  // 2: HRSEL: Host request input is the HREQ pin (when host req is enabled, PCS[1] is disabled
  // 1: HRPOL: HREQ pin is active high provided PCSPOL[1] is clear
  // 0: HREN: Host request is disabled
  SLAVE_CFGR0 = 0;

  // 27: PCSCFG - PCS[3:2] configured for chip select
  // 26: OUTCFG - Output data retains last value when chip select is negated
  // 25-24: PINCFG - SIN is used for input and SOUT is used for output
  // 18-16: MATCH - Match is disabled
  // 11-8: PCSPOL: Peripheral Chip Select is active low
  // 3: NOSTALL: Transfers stall when transmit FIFO is empty
  // 2: AUTOPCS: Automatic PCS generation is disabled
  // 1: SAMPLE: Ignored in slave mode
  // 0: MASTER: Slave mode
  //SLAVE_CFGR1 = 0; // cables are crossed.
  SLAVE_CFGR1 = 3 << 24; // cables are MOSI-MOSI & MISO - MISO
  Serial.print("CFG1");Serial.println(SLAVE_CFGR1, HEX);
  //SLAVE_CFGR1 = LPSPI_CFGR1_OUTCFG;

  // Clear status register
  // (=all interrupt flags except receive data flag and transmit data flag) 
  SLAVE_SR = 0x3F00; 

  // Update transfer params (set frame size)
  SLAVE_TCR_REFRESH;
  
  // dummy data, must populate initial TX slot
  SLAVE_TDR(1);
  SLAVE_TDR(2);
  SLAVE_TDR(3);
  SLAVE_TDR(4);
  SLAVE_TDR(5);
  SLAVE_TDR(6);
  SLAVE_TDR(7);
  SLAVE_TDR(8);

  
  // Enable Module, Debug Mode (LPSPI_CR_MEN = 1, LPSPI_CR_DBGEN = 1 << 3)
  // TODO:
  SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN | LPSPI_CR_DOZEN; /* Enable Module, Debug Mode, Doze Mode */

  // Enable LPSPI4 IRQ (35)
  NVIC_ENABLE_IRQ(nvic_irq);
  NVIC_SET_PRIORITY(nvic_irq, 1);
}

#endif
