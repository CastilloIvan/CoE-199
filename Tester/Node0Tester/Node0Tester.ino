// Setup Node Information
#define NODE_ID 0
#define MAX_NODES 3
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
  uint8_t psent;
  float rtt;
};
float SPEED = 0;
float LATITUDE = 0;
float LONGITUDE = 0;

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
routingCache ROUTINGCACHE[255];
uint8_t CACHEINDEX = 0;

// Setup Routing Table Structure
struct routingTable {
  uint8_t isRouted;
  uint8_t hopCount;
  uint8_t previousNode;
  uint8_t sourceNode;
  uint8_t nextNode;
};
routingTable ROUTINGTABLE[MAX_NODES];
uint8_t DATACOUNTER = 0;

// Sends DATA Packets
void SendDATAPacket(uint8_t PacketType, uint8_t IntermediateNode, uint8_t SourceNode, float Speed, float Latitude, float Longitude, uint8_t Psent, float Rtt) {
  dataPacket DATAPacket = {PacketType, IntermediateNode, SourceNode, Speed, Latitude, Longitude, Psent, Rtt};
  byte PacketBuffer[50];
  memcpy(PacketBuffer, &DATAPacket, sizeof(DATAPacket));
  LoRa.beginPacket();
  LoRa.write(PacketBuffer, sizeof(PacketBuffer));
  LoRa.endPacket();
}

// Sends AODV Packets
void SendAODVPacket(uint8_t PacketType, uint8_t BroadcastId, uint8_t HopCount, uint8_t PreviousNode, uint8_t SourceNode, uint8_t NextNode) {
  aodvPacket AODVPacket = {PacketType, BroadcastId, HopCount, PreviousNode, SourceNode, NextNode};
  byte PacketBuffer[50];
  memcpy(PacketBuffer, &AODVPacket, sizeof(AODVPacket));
  LoRa.beginPacket();
  LoRa.write(PacketBuffer, sizeof(PacketBuffer));
  LoRa.endPacket();
}

// For Testing
uint8_t PSENT = 1;
float TSENT = 0;
float TRECEIVED = 0;
float RTTTOTAL = 0;
uint8_t RTTCOUNTER = 0;
float RTT = 0;

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
  Serial.println("Starting Node " + String(NODE_ID) + ".");
  while(!LoRa.begin(433E6)) {
    Serial.println("Starting Node " + String(NODE_ID) + " Failed.");
    delay(10000);
  }
  Serial.println("Starting Node " + String(NODE_ID) + " Succeeded.");
}

void loop() {
  // Transmitter Mode (Lasts Briefly)
         if(ROUTINGTABLE[NODE_ID].isRouted == 1) {
    // Updates SPEED, LATITUDE, and LONGITUDE
    if(MCU.available() > 0) {
      GPS.encode(MCU.read());
      SPEED = GPS.speed.kmph();
      LATITUDE = GPS.location.lat();
      LONGITUDE = GPS.location.lng();
    }
    // Sends DATA Packet
    SendDATAPacket(DATA_PACKET, NODE_ID, NODE_ID, SPEED, LATITUDE, LONGITUDE, PSENT, RTT);
    DATACOUNTER += 1;
    PSENT += 1;
    TSENT = millis();
    Serial.println("Sent DATA Packet.");
  } else if(ROUTINGTABLE[NODE_ID].isRouted == 0) {
    // Sends RREQ Packet
    SendAODVPacket(RREQ_PACKET, CACHEINDEX, 0, NODE_ID, NODE_ID, NODE_ID);
    ROUTINGCACHE[CACHEINDEX] = {CACHEINDEX, 0, NODE_ID, NODE_ID, NODE_ID};
    CACHEINDEX += 1;
    if(CACHEINDEX >= 255) { CACHEINDEX = 0; }
    Serial.println("Sent RREQ Packet.");
  }

  // Receiver Mode (Lasts For 10 Seconds)
  long RECEIVINGTIME = millis();
  while(true) {
    int PACKETSIZE = LoRa.parsePacket();
    if(PACKETSIZE) {
      byte PACKETBUFFER[50];
      LoRa.readBytes(PACKETBUFFER, PACKETSIZE);
             if(PACKETBUFFER[0] == DATA_PACKET && PACKETBUFFER[1] != NODE_ID && PACKETBUFFER[2] != NODE_ID && ROUTINGTABLE[PACKETBUFFER[2]].isRouted == 1 && ROUTINGTABLE[PACKETBUFFER[2]].previousNode == PACKETBUFFER[1]) {
        // Forwards DATA Packet
        float speed;
        memcpy(&speed, &PACKETBUFFER[3], sizeof(speed));
        float latitude;
        memcpy(&latitude, &PACKETBUFFER[7], sizeof(latitude));
        float longitude;
        memcpy(&longitude, &PACKETBUFFER[11], sizeof(longitude));
        float rtt;
        memcpy(&rtt, &PACKETBUFFER[16], sizeof(rtt));
        SendDATAPacket(PACKETBUFFER[0], NODE_ID, PACKETBUFFER[2], speed, latitude, longitude, PACKETBUFFER[15], rtt);
      } else if(PACKETBUFFER[0] == RREQ_PACKET && PACKETBUFFER[1] < 255 && PACKETBUFFER[2] < MAX_NODES && PACKETBUFFER[3] != NODE_ID && PACKETBUFFER[4] != NODE_ID && PACKETBUFFER[5] != NODE_ID) {
        // Forwards RREQ Packet
        SendAODVPacket(PACKETBUFFER[0], CACHEINDEX, PACKETBUFFER[2] + 1, PACKETBUFFER[5], PACKETBUFFER[4], NODE_ID);
        ROUTINGCACHE[CACHEINDEX] = {CACHEINDEX, PACKETBUFFER[2] + 1, PACKETBUFFER[5], PACKETBUFFER[4], NODE_ID};
        CACHEINDEX += 1;
        if(CACHEINDEX >= 255) { CACHEINDEX = 0; }
      } else if(PACKETBUFFER[0] == RREP_PACKET && PACKETBUFFER[1] < 255 && PACKETBUFFER[2] < MAX_NODES && PACKETBUFFER[3] == NODE_ID && PACKETBUFFER[4] == NODE_ID && PACKETBUFFER[5] != NODE_ID) {
        // Receives RREP Packet
        ROUTINGTABLE[PACKETBUFFER[4]] = {1, PACKETBUFFER[2], ROUTINGCACHE[PACKETBUFFER[1]].previousNode, PACKETBUFFER[4], PACKETBUFFER[5]};
      } else if(PACKETBUFFER[0] == RREP_PACKET && PACKETBUFFER[1] < 255 && PACKETBUFFER[2] < MAX_NODES && PACKETBUFFER[3] == NODE_ID && PACKETBUFFER[4] != NODE_ID && PACKETBUFFER[5] != NODE_ID) {
        // Forwards RREP Packet
        SendAODVPacket(PACKETBUFFER[0], ROUTINGCACHE[PACKETBUFFER[1]].broadcastId, PACKETBUFFER[2], ROUTINGCACHE[PACKETBUFFER[1]].previousNode, PACKETBUFFER[4], NODE_ID);
        ROUTINGTABLE[PACKETBUFFER[4]] = {1, PACKETBUFFER[2], ROUTINGCACHE[PACKETBUFFER[1]].previousNode, PACKETBUFFER[4], PACKETBUFFER[5]};
      } else if(PACKETBUFFER[0] == RRER_PACKET && PACKETBUFFER[1] < 255 && PACKETBUFFER[2] < MAX_NODES && PACKETBUFFER[3] == NODE_ID && PACKETBUFFER[4] == NODE_ID && PACKETBUFFER[5] != NODE_ID) {
        // Receives RRER Packet
        ROUTINGTABLE[PACKETBUFFER[4]] = {0, 0, 0, 0, 0};
      } else if(PACKETBUFFER[0] == RRER_PACKET && PACKETBUFFER[1] < 255 && PACKETBUFFER[2] < MAX_NODES && PACKETBUFFER[3] == NODE_ID && PACKETBUFFER[4] != NODE_ID && PACKETBUFFER[5] != NODE_ID) {
        // Forwards RRER Packet
        SendAODVPacket(PACKETBUFFER[0], ROUTINGTABLE[PACKETBUFFER[4]].isRouted,    PACKETBUFFER[2], ROUTINGTABLE[PACKETBUFFER[4]].previousNode, PACKETBUFFER[4], NODE_ID);
        ROUTINGTABLE[PACKETBUFFER[4]] = {0, 0, 0, 0, 0};
      } else if(PACKETBUFFER[0] == DACK_PACKET && PACKETBUFFER[1] != NODE_ID && PACKETBUFFER[2] == NODE_ID && ROUTINGTABLE[PACKETBUFFER[2]].isRouted == 1 && ROUTINGTABLE[PACKETBUFFER[2]].nextNode == PACKETBUFFER[1]) {
        // Receives DACK Packet
        DATACOUNTER = 0;
        TRECEIVED = millis();
        RTTTOTAL += TRECEIVED - TSENT;
        RTTCOUNTER += 1;
        RTT = RTTTOTAL / RTTCOUNTER;
      } else if(PACKETBUFFER[0] == DACK_PACKET && PACKETBUFFER[1] != NODE_ID && PACKETBUFFER[2] != NODE_ID && ROUTINGTABLE[PACKETBUFFER[2]].isRouted == 1 && ROUTINGTABLE[PACKETBUFFER[2]].nextNode == PACKETBUFFER[1]) {
        // Forwards DACK Packet
        float speed;
        memcpy(&speed, &PACKETBUFFER[3], sizeof(speed));
        float latitude;
        memcpy(&latitude, &PACKETBUFFER[7], sizeof(latitude));
        float longitude;
        memcpy(&longitude, &PACKETBUFFER[11], sizeof(longitude));
        float rtt;
        memcpy(&rtt, &PACKETBUFFER[16], sizeof(rtt));
        SendDATAPacket(PACKETBUFFER[0], NODE_ID, PACKETBUFFER[2], speed, latitude, longitude, PACKETBUFFER[15], rtt);
      }
    }
    if(millis() - RECEIVINGTIME > 10000) { break; } else { continue; }
  }

  if(DATACOUNTER >= 3) {
    SendAODVPacket(RRER_PACKET, ROUTINGTABLE[NODE_ID].isRouted, ROUTINGTABLE[NODE_ID].hopCount, ROUTINGTABLE[NODE_ID].previousNode, ROUTINGTABLE[NODE_ID].sourceNode, ROUTINGTABLE[NODE_ID].nextNode);
    ROUTINGTABLE[NODE_ID] = {0, 0, 0, 0, 0};
    DATACOUNTER = 0;
  }
}