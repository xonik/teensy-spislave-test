#include "SPISlave_T4.h"
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

void setup() {
  Serial.begin(115200);

  Serial.print("Hello "); 
  mySPI.begin();
  Serial.print("World "); 
}

void loop() {
  delay(300);
}

/*
 * Master
 */
/*
#include <SPI.h>
SPISettings settings(10000000, MSBFIRST, SPI_MODE0);

void setup() {
  
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; 

  Serial.begin(1000000);
  pinMode (10, OUTPUT);
  digitalWriteFast(10,HIGH);
  Serial.println("SPI Starting");

  SPI.begin();   
  SPI.beginTransaction(settings);  
}

void loop() {
  digitalWriteFast(1,HIGH);
  digitalWriteFast(1,LOW); 

  delay(500);

  digitalWriteFast(10,LOW);
  Serial.println(SPI.transfer(0xAA));
  digitalWriteFast(10,HIGH);  
}
*/ 
