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
