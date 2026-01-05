#include <Arduino.h>

HardwareSerial raspi(PA3, PA2);

const int MAXBYTE = 10 * 1000;
byte byteArr[MAXBYTE];
uint32_t lastRxTime;

void setup() {
  Serial.begin(115200);      // USB debug
  raspi.begin(38400);     // Hardware UART

  while(!Serial);

  Serial.println("STM32 UART RX packet size test");
}

void loop() {
  while (raspi.available()) {
    Serial.println("RX BUFFER SIZE: " + String(SERIAL_RX_BUFFER_SIZE));
    int n = raspi.readBytes(byteArr,MAXBYTE);
    Serial.println(n);
    
    for(int i  = 0;i < n;i++){
      Serial.print(char(byteArr[i]));
    }
    if(byteArr[0] == byte('G')) raspi.println("GS,1,1,1");
  }

  if(millis() - 5000 > lastRxTime && millis() > 5000){
    lastRxTime = millis();
    raspi.println("PACKET_PLEASE");
    Serial.println("PACKET_PLEASE");
  }
}
