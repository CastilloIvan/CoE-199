// Setup Node Information
#define NODE_ID 5
#define MAX_NODES 5
#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RRER_PACKET 3
#define BEAT_PACKET 4

// Setup GPS Module
#include<SoftwareSerial.h>
#include<TinyGPSPlus.h>
SoftwareSerial MCU(4, 3);
TinyGPSPlus GPS;

// Setup LoRa Module
#include<SPI.h>
#include<LoRa.h>
#define SS    10
#define RESET 9
#define DIO0  2
#define TX_POWER          11
#define SPREADING_FACTOR  9
#define CODING_RATE       5
#define SIGNAL_BANDWIDTH  125E3
#define CARRIER_FREQUENCY 433E6

// Setup AODV Packet Structure
struct aodvPacket {
  uint8_t packetType;
  uint8_t broadcastId;
  uint8_t hopCount;
  uint8_t previousNode;
  uint8_t sourceNode;
  uint8_t nextNode;
};

// Setup Routing Table Structure
struct routingTable {
  uint8_t isRouted;
  uint8_t hopCount;
  uint8_t previousNode;
  uint8_t sourceNode;
  uint8_t nextNode;
};
routingTable RoutingTable[MAX_NODES];
long lifeTime = 0;

// Sends AODV Packets
void sendAODVPacket(uint8_t packetType, uint8_t broadcastId, uint8_t hopCount, uint8_t previousNode, uint8_t sourceNode, uint8_t nextNode) {
  aodvPacket AODVPacket = {packetType, broadcastId, hopCount, previousNode, sourceNode, nextNode};
  byte packetBuffer[50];
  memcpy(packetBuffer, &AODVPacket, sizeof(AODVPacket));
  LoRa.beginPacket();
  LoRa.write(packetBuffer, sizeof(packetBuffer));
  LoRa.endPacket();
}

void setup() {
  // Setup GPS Module
  Serial.begin(9600);
  MCU.begin(9600);

  // Setup LoRa Module
  /*
  LoRa.setPins(SS, RESET, DIO0);
  LoRa.setTxPower(TX_POWER);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);
  */

  // Tests Setup
  Serial.println("Starting Node " + String(NODE_ID));
  while(!LoRa.begin(433E6)) {
    Serial.println("Starting Node " + String(NODE_ID) + " Failed");
    delay(10000);
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if(packetSize) {
    byte packetBuffer[50];
    LoRa.readBytes(packetBuffer, packetSize);
    Serial.println("Packet Type:        " + String(packetBuffer[0]));
    Serial.println("Intermediate Node:  " + String(packetBuffer[1]));
    Serial.println("Source Node:        " + String(packetBuffer[2]));
    float speed;
    memcpy(&speed, &packetBuffer[3], sizeof(speed));
    float latitude;
    memcpy(&latitude, &packetBuffer[7], sizeof(latitude));
    float longitude;
    memcpy(&longitude, &packetBuffer[11], sizeof(longitude));    
    Serial.println("Speed:              " + String(speed, 10));
    Serial.println("Latitude:           " + String(latitude, 10));
    Serial.println("Longitude:          " + String(longitude, 10));
    Serial.println("RSSI:               " + String(LoRa.packetRssi()));
  }
}