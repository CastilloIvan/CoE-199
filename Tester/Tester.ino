// Setup Node Information
#define NODE_ID 0
#define MAX_NODES 5
#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RRER_PACKET 3

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

// Setup Routing Cache Structure
struct routingCache {
  int routingId;
  int sourceNode;
  int previousNode;
  int nextNode;
  int destinationNode;
  int hopCount;
};
routingCache RoutingCache[65536];

// Setup Routing Table Structure
struct routingTable {
  bool isRouted;
  int sourceNode;
  int previousNode;
  int nextNode;
  int destinationNode;
  int hopCount;
};
routingTable RoutingTable[MAX_NODES];

// Setup AODV Packet Structure
struct aodvPacket {
  uint8_t packetType;
  int routingId;
  int sourceNode;
  int destinationNode;
  int hopCount;
};

// Setup Data Packet Structure
struct dataPacket {
  uint8_t packetType;
  int sourceNode;
  double latitude;
  double longitude;
  double speed;
};

// Sends Data Packets
void sendDataPacket(TinyGPSPlus GPS) {
  dataPacket DataPacket = {DATA_PACKET, NODE_ID, GPS.location.lat(), GPS.location.lng(), GPS.speed.kmph()};
  LoRa.beginPacket();
  LoRa.write((byte*)&DataPacket, sizeof(DataPacket));
  LoRa.endPacket();
}

// Sends RREQ Packets
void sendRREQPacket() {
  aodvPacket AODVPacket = {RREQ_PACKET, rand() % 65536, NODE_ID, MAX_NODES, 0};
  LoRa.beginPacket();
  LoRa.write((byte*)&AODVPacket, sizeof(AODVPacket));
  LoRa.endPacket();
}

void setup() {
  // Setup GPS Module
  Serial.begin(9600);
  MCU.begin(9600);

  // Setup LoRa Module
  LoRa.setPins(SS, RESET, DIO0);
  LoRa.setTxPower(TX_POWER);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);

  // Setup Routing Table
  for(int i = 0; i < MAX_NODES; i++) {
    RoutingTable[i] = {false, i, i, i, MAX_NODES, 0};
  }

  // Tests Setup
  Serial.println("LoRa Receiver");
  if(!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa Receiver failed!");
    while (1);
  }
}

void loop() {
  // Transmitter Mode (Lasts Briefly)
  if(RoutingTable[NODE_ID].isRouted) {
    // Sends Data Packet
    if(MCU.available() > 0) {
      GPS.encode(MCU.read());
      sendDataPacket(GPS);
    }
  } else {
    // Sends RREQ Packet
    sendRREQPacket();
  }

  // Receiver Mode (Lasts for 10 seconds)
  long startTime = millis();
  long elapsedTime = 0;
  while(elapsedTime < 10000) {
    // Receive RREP Packet Here
    // Receive RRER Packet Here
    // Forward Data Packet Here
    // Forward RREQ Packet Here
    // Forward RREP Packet Here
    // Forward RRER Packet Here
    elapsedTime = millis() - startTime;
  }
  
  /*
  int packetSize = LoRa.parsePacket();
  if(packetSize) {
    byte packetBuffer[256];
    LoRa.readBytes(packetBuffer, packetSize);
    dataPacket DataPacket;
    memcpy(&DataPacket, packetBuffer, sizeof(DataPacket));
    Serial.println("Received Data Packet:");
    Serial.println("Packet Type: " + String(DataPacket.packetType));
    Serial.println("Source Node: " + String(DataPacket.sourceNode));
    Serial.println("Latitude:    " + String(DataPacket.latitude));
    Serial.println("Longitude:   " + String(DataPacket.longitude));
    Serial.println("Speed:       " + String(DataPacket.speed));
    Serial.println("RSSI:        " + String(LoRa.packetRssi()));
  }
  */
}