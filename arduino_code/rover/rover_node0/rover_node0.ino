#include <SPI.h>
#include <LoRa.h>
//#include "SAMDTimerInterrupt.h"

int counter = 0;
int header = 0xAA;

// how often to ping the base node
int sendInterval = 5000;
// timestamp of last message sent
int lastSendTime = 0;

byte rxHeader = 0xFF;
int rxCounter = 0;

unsigned long propDelay = 0;

// Init SAMD timer TIMER_TC3
//SAMDTimer ITimer(TIMER_TC3);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.begin(868E6);
  delay(1000);

//  while (!LoRa.begin(868E6)) {
//    Serial.println("Starting LoRa failed!");
//    Serial.println("Retrying...");
//    delay(500);
//  }
}

void loop() {
  if (millis() - lastSendTime > sendInterval) {
    sendPacket();
    lastSendTime = millis(); 
  }
  if (LoRa.parsePacket()) {
    propDelay = millis() - (unsigned long)lastSendTime;
    receivePacket();
    //if(rxHeader == header) {
    Serial.print("propagation delay: ");
    Serial.println(propDelay);
    //}
  }
}

void sendPacket() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print(header);
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

}

void receivePacket() {
  
  rxHeader = (int)LoRa.read();
  rxCounter = (int)LoRa.read();
  Serial.print("Received: ");
  Serial.print(rxHeader, HEX);
  Serial.print(" ");
  Serial.print(rxCounter, DEC);
  // print RSSI of packet
  Serial.print(" with RSSI ");
  Serial.println(LoRa.packetRssi());
}
