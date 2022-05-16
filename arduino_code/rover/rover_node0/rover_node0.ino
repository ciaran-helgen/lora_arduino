#include <SPI.h>
#include <LoRa.h>

int counter = 0;
char header = '0';

int sendInterval = 5000;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  while (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    Serial.println("Retrying...");
    delay(500);
  }
}

void loop() {
  if (millis() - lastSendTime > sendInterval) {
    sendPacket();
  }
  if (LoRa.parsePacket()) {
    receivePacket();
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
  
  (char)LoRa.read();
  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}
