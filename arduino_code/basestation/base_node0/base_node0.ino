#include <SPI.h>
#include <LoRa.h>

int counter = 0;
char header = '0';

char rxHeader = '^';
int rxCounter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  while (!LoRa.begin(868E6)) {
      Serial.println("Starting LoRa failed!");
      Serial.println("Retrying...");
      delay(500);
    }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  //relay the message
  if (packetSize) {
    receivePacket();
    sendPacket();
  }
}

void receivePacket() {
  
  (char)LoRa.read();
  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}

void sendPacket() {
  Serial.print("Sending packet: ");
  Serial.println(rxCounter);

  // send packet
  LoRa.beginPacket();
  LoRa.print(rxHeader);
  LoRa.print(rxCounter);
  LoRa.endPacket();
}
