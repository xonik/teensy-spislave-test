#if !defined(_SPISlave_T4_H_)
#define _SPISlave_T4_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include <SPI.h>

uint8_t outbyte = 0;

typedef enum SPI_BITS {
  SPI_8_BITS = 8,
  SPI_16_BITS = 16,
  SPI_32_BITS = 32,
} SPI_BITS;

typedef void (*_SPI_ptr)();

#define SPISlave_T4_CLASS template<SPIClass* port = nullptr, SPI_BITS bits = SPI_8_BITS>
#define SPISlave_T4_FUNC template<SPIClass* port, SPI_BITS bits>
#define SPISlave_T4_OPT SPISlave_T4<port, bits>

extern SPIClass SPI;

class SPISlave_T4_Base {
  public:
    virtual void SLAVE_ISR();
};

static SPISlave_T4_Base* _LPSPI4 = nullptr;

SPISlave_T4_CLASS class SPISlave_T4 : public SPISlave_T4_Base {
  public:
    SPISlave_T4();
    void begin();
    uint32_t transmitErrors();
    void srStatus();

  private:
    _SPI_ptr _spihandler = nullptr;
    void SLAVE_ISR();
    int _portnum = 0;
    uint32_t nvic_irq = 0;
    uint32_t transmit_errors = 0;
    bool sniffer_enabled = 0;
};

// 0x40394000 is LPSPI1 memory map base address (48.5.1.1 in manual, p2867)
// - Offset is in 8bit blocks whereas number here is in 32 bit blocks
#define SLAVE_PARAM spiAddr[1] // 4 = 0x04, Parameter (PARAM)
#define SLAVE_CR spiAddr[4] // 16 = 0x10, Control (CR)
#define SLAVE_FCR spiAddr[22] //88 = 0x58, FIFO Control
#define SLAVE_IER spiAddr[6] // 24 = 0x18, Interrupt Enable
#define SLAVE_CFGR0 spiAddr[8] // 32 = 0x20, Configuration 0
#define SLAVE_CFGR1 spiAddr[9] // 36 = 0x24, Configuration 1
#define SLAVE_TCR spiAddr[24] // 96 = 0x60 
#define SLAVE_TDR spiAddr[25] // 100 = 0x64, Transmit Data
#define SLAVE_RDR spiAddr[29] // 116 = 0x74, Receive Data
#define SLAVE_SR spiAddr[5] // 20 = 0x14, Status (SR) 
#define SLAVE_FSR spiAddr[23] // 92 = 0x5C, FIFO Status

// 96 = 0x60, Transmit Command
// Clock Pol = low
// Clock Phase = capture leading edge, change on following edge
// Prescale = 1
// Men hva med PCS, LSB First, Byte Swap, Cont, Contc, rxmsk, txmsk, width? Antakelig 0?
// Frame size = 7 (= 8bit transfers)
#define SLAVE_TCR_REFRESH spiAddr[24] = (0UL << 27) | LPSPI_TCR_FRAMESZ(bits - 1)

#define SLAVE_PORT_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x40394000 + (0x4000 * _portnum)))
#define SLAVE_PINS_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x401F84EC + (_portnum * 0x10)))

 
void lpspi4_slave_isr() {
  _LPSPI4->SLAVE_ISR();
}


SPISlave_T4_FUNC SPISlave_T4_OPT::SPISlave_T4() {
  if ( port == &SPI ) {
    _LPSPI4 = this;
    _portnum = 3;

    // CCM Clock Gating Register 1 - CG3 - LPSPI4 Clocks enable
    CCM_CCGR1 |= (3UL << 6);

    // Interrupt vector for LPSPI
    nvic_irq = 32 + _portnum;

    // Interrupt Service Routine
    _VectorsRam[16 + nvic_irq] = lpspi4_slave_isr;

    /* Alternate pins not broken out on Teensy 4.0/4.1 for LPSPI4 */
    SLAVE_PINS_ADDR;

    //NB: NOT THE SAME SPIADDR AS FOR OTHERS, THIS IS PINS!
    spiAddr[0] = 0; /* PCS0_SELECT_INPUT */
    spiAddr[1] = 0; /* SCK_SELECT_INPUT */
    spiAddr[2] = 0; /* SDI_SELECT_INPUT */
    spiAddr[3] = 0; /* SDO_SELECT_INPUT */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
  } 
}

// Definitions
// Word: 0-padded 32 bit.
// Frame: A number of bits in a total transfer, may be more than
//        1 word, in which case frame is split into multiple words.
//        As long as PCS is asserted (low). Divisible by 2

// Interrupts:
// TDF: Data can be written to transmit FIFO
#define SR_TDF (1 << 0)
#define IRQ_TDIE (1 << 0)
// RDF: Data can be read from receive FIFO
#define SR_RDF (1 << 1)
#define IRQ_RDIE (1 << 1)
// WCF: Word complete: When enough bytes have been transferred to fill a word
#define SR_WCF (1 << 8)
#define IRQ_WCIE (1 << 8)
// FCF: Frame complete - When PCS goes high.
#define SR_FCF (1 << 9)
#define IRQ_FCIE (1 << 9)
// TCF: Transfer Complete - Used for send. When all data in transmit/command FIFO is empty.
#define SR_TCF (1 << 10)
#define IRQ_TCIE (1 << 10)
// TEF: Transmit/command FIFO underrun
#define SR_TEF (1 << 11)
#define IRQ_TEIE (1 << 11)
// REF: Receive FIFO overflow
#define SR_REF (1 << 12)
#define IRQ_REIE (1 << 12)
// DMF: Data has matched configured data match value
#define SR_DMF (1 << 13)
#define IRQ_DMIE (1 << 13)

// Not an interrupt: Module busy status flag
#define SR_MBF (1 << 24)

SPISlave_T4_FUNC void SPISlave_T4_OPT::srStatus() {
  SLAVE_PORT_ADDR;
  if(SLAVE_SR & SR_MBF) Serial.print("Module busy, "); // Not an interrupt
  if(SLAVE_SR & SR_DMF) Serial.print("Data matched, ");
  if(SLAVE_SR & SR_REF) Serial.print("RX FIFO overflow, ");
  if(SLAVE_SR & SR_TEF) Serial.print("TX FIFO underrun, ");
  if(SLAVE_SR & SR_TCF) Serial.print("All TX completed, ");
  if(SLAVE_SR & SR_FCF) Serial.print("Frame completed, ");
  if(SLAVE_SR & SR_WCF) Serial.print("Word completed, ");
  if(SLAVE_SR & SR_RDF) Serial.print("RX data ready, ");
  if(SLAVE_SR & SR_TDF) Serial.print("TX data ready, ");
  
  Serial.print("TXFIFO: ");
  Serial.print(SLAVE_FSR & 0xFF);
  
  Serial.print(", RXFIFO: ");
  Serial.println(SLAVE_FSR >> 16);
  
  /*
  0000 1010 0000 0001
  TEF - FIFO underrun has occured
  FCF - Frame transfer has completed
  TDF - Transmit Data Flag, words in transmit FIFO is equal or less than TX Water Mark
  */
}

uint32_t repeat = 0;

SPISlave_T4_FUNC void SPISlave_T4_OPT::SLAVE_ISR() {

  SLAVE_PORT_ADDR;
  Serial.print("\nSlave ISR: ");    
  Serial.print("My Func callback ");Serial.print(repeat++);Serial.print(", time ");Serial.println(millis());

  srStatus();
 
  // Receive
  if((SLAVE_FSR >> 16) > 1) {
    uint32_t data = SLAVE_RDR;
    Serial.print("Received value: "); Serial.println(data);
    data = SLAVE_RDR;
    Serial.print("Received value: "); Serial.println(data);  

    // Transmit - We must always put a byte per receive into the TX FIFO
    // if not we get buffer underrun and nothing is sent or received.
    SLAVE_TDR = 0;
    SLAVE_TDR = 0;    
  }
  
  // Clear all flags.
  SLAVE_SR = 0x3F03;
  asm volatile ("dsb");
}

// Interrupt trigges bare i takt med input dersom det kommer to bytes
// samtidig!
SPISlave_T4_FUNC void SPISlave_T4_OPT::begin() {
  SLAVE_PORT_ADDR;

  // 2, Software reset Reset SPI Module
  SLAVE_CR = LPSPI_CR_RST;

  // Disable everything but especially 0: MEN - Module enable
  SLAVE_CR = 0;
  SLAVE_TCR_REFRESH;
  
  // If FCR is 10001, SR = 1, if FCR = 0, SR = 2 when entering ISR
  //SLAVE_FCR = 0x00010001; /* if set to 0x10001: 1x watermark for RX and TX */
  SLAVE_FCR = 0;

  // Enabled interrupts
  SLAVE_IER = IRQ_FCIE;
  
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
  SLAVE_CFGR1 = 0 | (1 << 3);

  // Enable Module, Debug Mode (LPSPI_CR_MEN = 1, LPSPI_CR_DBGEN = 1 << 3)
  SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN; 

  // Clear status register
  // (=all interrupt flags except receive data flag and transmit data flag) 
  SLAVE_SR = 0x3F00; 
 
   /// dummy data, must populate initial TX slot
  SLAVE_TDR = 0;
  SLAVE_TDR = 0;

  // Enable LPSPI4 IRQ (35)
  NVIC_ENABLE_IRQ(nvic_irq);
  NVIC_SET_PRIORITY(nvic_irq, 1);

  Serial.print("Params: ");
  Serial.println(SLAVE_PARAM, BIN); // Defaults to 16 words in both FIFOs
}
#endif
