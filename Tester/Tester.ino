// Setup Node Information
#define NODE_ID 0
#define MAX_NODES 5
#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RRER_PACKET 3
#define DACK_PACKET 4

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

// Setup DATA Packet Structure
struct dataPacket {
  uint8_t packetType;
  uint8_t intermediateNode;
  uint8_t sourceNode;
  float speed;
  float latitude;
  float longitude;
};

// Setup AODV Packet Structure
struct aodvPacket {
  uint8_t packetType;
  uint8_t broadcastId;
  uint8_t hopCount;
  uint8_t previousNode;
  uint8_t sourceNode;
  uint8_t nextNode;
};

// Setup Routing Cache Structure
struct routingCache {
  uint8_t broadcastId;
  uint8_t hopCount;
  uint8_t previousNode;
  uint8_t sourceNode;
  uint8_t nextNode;
};
routingCache RoutingCache[200];
uint8_t cacheIndex = 0;

// Setup Routing Table Structure
struct routingTable {
  uint8_t isRouted;
  uint8_t dackCounter;
  uint8_t previousNode;
  uint8_t sourceNode;
  uint8_t nextNode;
};
routingTable RoutingTable[MAX_NODES];

// Sends DATA Packets
void sendDataPacket(uint8_t packetType, uint8_t intermediateNode, uint8_t sourceNode, float speed, float latitude, float longitude) {
  dataPacket DataPacket = {packetType, intermediateNode, sourceNode, speed, latitude, longitude};
  byte packetBuffer[50];
  memcpy(packetBuffer, &DataPacket, sizeof(DataPacket));
  LoRa.beginPacket();
  LoRa.write(packetBuffer, sizeof(packetBuffer));
  LoRa.endPacket();
}

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
  // Transmitter Mode (Lasts Briefly)
         if(RoutingTable[NODE_ID].isRouted == 1 && MCU.available() > 0) {
    // Sends DATA Packet
    GPS.encode(MCU.read());
    sendDataPacket(DATA_PACKET, NODE_ID, NODE_ID, GPS.speed.kmph(), GPS.location.lat(), GPS.location.lng());
    RoutingTable[NODE_ID].dackCounter += 1;
    Serial.println("DATA Packet Sent.");
  } else if(RoutingTable[NODE_ID].isRouted == 0) {
    // Sends RREQ Packet
    sendAODVPacket(RREQ_PACKET, cacheIndex, 0, NODE_ID, NODE_ID, NODE_ID);
    RoutingCache[cacheIndex] = {cacheIndex, 0, NODE_ID, NODE_ID, NODE_ID};
    cacheIndex++;
    Serial.println("RREQ Packet Sent.");
  }

  // Receiver Mode (Lasts For 10 Seconds)
  long receivingTime = millis();
  while(true) {
    int packetSize = LoRa.parsePacket();
    if(packetSize) {
      byte packetBuffer[50];
      LoRa.readBytes(packetBuffer, packetSize);
             if(packetBuffer[0] == DATA_PACKET && packetBuffer[1] != NODE_ID && packetBuffer[2] != NODE_ID  && RoutingTable[packetBuffer[2]].isRouted == 1 && RoutingTable[packetBuffer[2]].previousNode == packetBuffer[1]) {
        // Forwards DATA Packet
        float speed;
        memcpy(&speed, &packetBuffer[3], sizeof(speed));
        float latitude;
        memcpy(&latitude, &packetBuffer[7], sizeof(latitude));
        float longitude;
        memcpy(&longitude, &packetBuffer[11], sizeof(longitude));
        sendDataPacket(packetBuffer[0], NODE_ID, packetBuffer[2], speed, latitude, longitude);
        sendDataPacket(DACK_PACKET, packetBuffer[1], packetBuffer[2], speed, latitude, longitude);
      } else if(packetBuffer[0] == RREQ_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] != NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
        // Forwards RREQ Packet
        sendAODVPacket(packetBuffer[0], cacheIndex, packetBuffer[2] + 1, packetBuffer[5], packetBuffer[4], NODE_ID);
        RoutingCache[cacheIndex] = {cacheIndex, packetBuffer[2] + 1, packetBuffer[5], packetBuffer[4], NODE_ID};
        cacheIndex++;
      } else if(packetBuffer[0] == RREP_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] == NODE_ID && packetBuffer[5] != NODE_ID) {
        // Receives RREP Packet
        RoutingTable[packetBuffer[4]] = {true, 0, RoutingCache[packetBuffer[1]].previousNode, RoutingCache[packetBuffer[1]].sourceNode, RoutingCache[packetBuffer[1]].nextNode};
      } else if(packetBuffer[0] == RREP_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
        // Forwards RREP Packet
        sendAODVPacket(packetBuffer[0], RoutingCache[packetBuffer[1]].broadcastId, RoutingCache[packetBuffer[1]].hopCount, RoutingCache[packetBuffer[1]].previousNode, RoutingCache[packetBuffer[1]].sourceNode, RoutingCache[packetBuffer[1]].nextNode);
        RoutingTable[packetBuffer[4]] = {true, 0, RoutingCache[packetBuffer[1]].previousNode, RoutingCache[packetBuffer[1]].sourceNode, RoutingCache[packetBuffer[1]].nextNode};
      } else if(packetBuffer[0] == RRER_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] == NODE_ID && packetBuffer[5] != NODE_ID) {
        // Receives RRER Packet
        RoutingTable[packetBuffer[4]].isRouted = false;
      } else if(packetBuffer[0] == RRER_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
        // Forwards RRER Packet
        sendAODVPacket(packetBuffer[0], RoutingTable[packetBuffer[1]].isRouted, RoutingTable[packetBuffer[1]].dackCounter, RoutingTable[packetBuffer[1]].previousNode, RoutingTable[packetBuffer[1]].sourceNode, RoutingTable[packetBuffer[1]].nextNode);
        RoutingTable[packetBuffer[4]].isRouted = false;
      } else if(packetBuffer[0] == DACK_PACKET && packetBuffer[1] == NODE_ID && packetBuffer[2] == NODE_ID  && RoutingTable[packetBuffer[2]].isRouted == 1 && RoutingTable[packetBuffer[2]].previousNode == packetBuffer[1]) {
        // Receives DACK Packet
        RoutingTable[NODE_ID].dackCounter = 0;
        float speed;
        memcpy(&speed, &packetBuffer[3], sizeof(speed));
        float latitude;
        memcpy(&latitude, &packetBuffer[7], sizeof(latitude));
        float longitude;
        memcpy(&longitude, &packetBuffer[11], sizeof(longitude));
        Serial.println("Received DACK Packet:");
        Serial.println("Packet Type:        " + String(packetBuffer[0]));
        Serial.println("Intermediate Node:  " + String(packetBuffer[1]));
        Serial.println("Source Node:        " + String(packetBuffer[2]));    
        Serial.println("Speed:              " + String(speed, 10));
        Serial.println("Latitude:           " + String(latitude, 10));
        Serial.println("Longitude:          " + String(longitude, 10));
        Serial.println("RSSI:               " + String(LoRa.packetRssi()));
      }
    }
    if(millis() - receivingTime > 10000) {
      break;
    } else {
      continue;
    }
  }

  if(cacheIndex >= 255) {
    cacheIndex = 0;
  }
  if(RoutingTable[NODE_ID].dackCounter >= 3) {
    RoutingTable[NODE_ID].isRouted = 0;
  }
}