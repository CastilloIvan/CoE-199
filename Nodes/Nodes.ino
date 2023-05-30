// Setup Node Information
#define NODE_ID 0
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

// Setup Data Packet Structure
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
routingCache RoutingCache[255];
uint8_t cacheIndex = 0;

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

// Sends Data Packets
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
    // Sends Data Packet
    GPS.encode(MCU.read());
    sendDataPacket(DATA_PACKET, NODE_ID, NODE_ID, GPS.speed.kmph(), GPS.location.lat(), GPS.location.lng());
  } else if(RoutingTable[NODE_ID].isRouted == 0) {
    // Sends RREQ Packet
    sendAODVPacket(RREQ_PACKET, cacheIndex, 0, NODE_ID, NODE_ID, NODE_ID);
    RoutingCache[cacheIndex] = {cacheIndex, 0, NODE_ID, NODE_ID, NODE_ID};
    cacheIndex++;
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
      } else if(packetBuffer[0] == RREQ_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] != NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
        // Forwards RREQ Packet
        sendAODVPacket(packetBuffer[0], cacheIndex, packetBuffer[2] + 1, packetBuffer[5], packetBuffer[4], NODE_ID);
        RoutingCache[cacheIndex] = {cacheIndex, packetBuffer[2] + 1, packetBuffer[5], packetBuffer[4], NODE_ID};
        cacheIndex++;
      } else if(packetBuffer[0] == RREP_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] == NODE_ID && packetBuffer[5] != NODE_ID) {
        // Receives RREP Packet
        RoutingTable[packetBuffer[4]] = {true, RoutingCache[packetBuffer[1]].hopCount, RoutingCache[packetBuffer[1]].previousNode, RoutingCache[packetBuffer[1]].sourceNode, RoutingCache[packetBuffer[1]].nextNode};
        cacheIndex = 0;
      } else if(packetBuffer[0] == RREP_PACKET && 0 < packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
        // Forwards RREP Packet
        sendAODVPacket(packetBuffer[0], RoutingCache[packetBuffer[1]].broadcastId, RoutingCache[packetBuffer[1]].hopCount, RoutingCache[packetBuffer[1]].previousNode, RoutingCache[packetBuffer[1]].sourceNode, RoutingCache[packetBuffer[1]].nextNode);
        RoutingTable[packetBuffer[4]] = {true, RoutingCache[packetBuffer[1]].hopCount, RoutingCache[packetBuffer[1]].previousNode, RoutingCache[packetBuffer[1]].sourceNode, RoutingCache[packetBuffer[1]].nextNode};
      } else if(packetBuffer[0] == RRER_PACKET && packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] == NODE_ID && packetBuffer[5] != NODE_ID) {
        // Receives RRER Packet
        RoutingTable[packetBuffer[4]].isRouted = false;
        cacheIndex = 0;
      } else if(packetBuffer[0] == RRER_PACKET && packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
        // Forwards RRER Packet
        aodvPacket AODVPacket;
        memcpy(&AODVPacket, packetBuffer, sizeof(AODVPacket));
        sendAODVPacket(RRER_PACKET, RoutingTable[packetBuffer[1]].isRouted, RoutingTable[packetBuffer[1]].hopCount - 1, RoutingTable[packetBuffer[1]].previousNode, RoutingTable[packetBuffer[1]].sourceNode, RoutingTable[packetBuffer[1]].nextNode);
        RoutingTable[packetBuffer[4]].isRouted = false;
      } else if(packetBuffer[0] == BEAT_PACKET && packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[3] == NODE_ID && packetBuffer[4] == NODE_ID && packetBuffer[5] == NODE_ID) {
        // Receives BEAT Packet
        lifeTime = millis();
        cacheIndex = 0;
      } else if(packetBuffer[0] == BEAT_PACKET && packetBuffer[1] < 255 && 0 < packetBuffer[2] < MAX_NODES && packetBuffer[4] != NODE_ID && RoutingTable[packetBuffer[4]].isRouted == 1 && RoutingTable[packetBuffer[4]].previousNode == packetBuffer[4]) {
        // Forwards BEAT Packet
        aodvPacket AODVPacket;
        memcpy(&AODVPacket, packetBuffer, sizeof(AODVPacket));
        sendAODVPacket(BEAT_PACKET, AODVPacket.broadcastId, AODVPacket.hopCount + 1, AODVPacket.previousNode, AODVPacket.sourceNode, AODVPacket.nextNode);
      }
    }
    if(millis() - receivingTime > 10000) {
      break;
    } else {
      continue;
    }
  }
}

/*
dataPacket DataPacket = {DATA_PACKET, NODE_ID, 14.6472606659, 121.0637359619, 1.7964400291};
byte packetBuffer[256];
memcpy(packetBuffer, &DataPacket, sizeof(DataPacket));
LoRa.beginPacket();
LoRa.write(packetBuffer, sizeof(packetBuffer));
LoRa.endPacket();
Serial.println("Data Packet Sent.");
delay(5000);

dataPacket DataPacket = {1, 2, 3, 1.7964400291, 14.6472606659, 121.0637359619};
byte packetBuffer[255];
memcpy(packetBuffer, &DataPacket, sizeof(DataPacket));
for(int i = 0; i < 255; i++) {
  printf("%x ", packetBuffer[i]);
}
double speed;
memcpy(&speed, &packetBuffer[8], sizeof(double));
printf("%f ", speed);
return 0;
*/