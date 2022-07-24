#include "SPISlave_T4.h"
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

void setup() {
  Serial.begin(115200);

  Serial.print("Hello "); 
  //mySPI.swapPins(0);
  //mySPI.onReceive(myFunc);  
  mySPI.begin();
  Serial.print("World "); 
}

void loop() {
  /*Serial.print("\nMillis: "); 
  Serial.println(millis());
  Serial.println();*/
  delay(1000);
}

/*
void myFunc() {
  Serial.print("My Func callback ");
  Serial.print(repeat++);
  Serial.print(", time "); 
  Serial.println(millis());
  mySPI.srStatus();
  
  uint8_t arr[] = { 3, 2, 8, 3, 10, 11, 33, 13, 14 };
  uint8_t i = 0;

  // With this while, program blocks in ISR!
  //while ( mySPI.active() ) { // Frame Complete Flag not set
    //if (mySPI.available()) { // Word Complete Flag set      
      if ( i++ > sizeof(arr) ) i = 0;
      // Without this, SR is 3 in ISR 
      Serial.print("Sending value: ");
      Serial.println(arr[i], HEX);
      //mySPI.pushr(arr[i]);
      while(
      Serial.print("Received value: ");
      //Without this, stuff works.
      Serial.println(mySPI.popr());
      Serial.println(mySPI.popr());
    //}
  //}
  Serial.println("Callback ended");
}
*/
