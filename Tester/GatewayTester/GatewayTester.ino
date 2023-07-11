// Setup Node Information
#define NODE_ID 3
#define MAX_NODES 3
#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RRER_PACKET 3
#define DACK_PACKET 4

// Setup GPS Module
//#include<SoftwareSerial.h>
//#include<TinyGPSPlus.h>
//SoftwareSerial MCU(4, 3);
//TinyGPSPlus GPS;

// Setup LoRa Module
#include<SPI.h>
#include<LoRa.h>
#define SCK   5
#define MISO  19
#define MOSI  27
#define SS    18
#define RESET 14
#define DIO0  26
#define TX_POWER          11
#define SPREADING_FACTOR  9
#define CODING_RATE       5
#define SIGNAL_BANDWIDTH  125E3
#define CARRIER_FREQUENCY 433E6

// Setup Wifi Module
#include <WiFi.h>
#define WIFI_AP       "GTE 3RD FLR"
#define WIFI_PASSWORD "gte646464"
#define TOKEN         "OBE_GATEWAY_TOKEN"
WiFiClient wifiClient;
int status = WL_IDLE_STATUS;
unsigned long lastSend;

// Setup ThingsBoard Information
#include <ThingsBoard.h>
char thingsboardServer[] = "thingsboard.cloud";
ThingsBoard tb(wifiClient);

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
routingCache ROUTINGCACHE[25];
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

// Initializes WiFi
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

// Reconnects to ThingsBoard
void Reconnect() {
  while(!tb.connected()) {
    if(WiFi.status() != WL_CONNECTED) { InitWiFi(); }
    Serial.print("Connecting to ThingsBoard node ...");
    if(tb.connect(thingsboardServer, TOKEN)) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      delay(5000);
    }
  }
}

// Uploads Data to ThingsBoard
void UploadData(uint8_t Node_Id, float Speed, float Latitude, float Longitude, float Pdr, float Rtt) {
         if(Node_Id == 0) {
    tb.sendTelemetryFloat("N0 Speed", Speed);
    tb.sendTelemetryFloat("N0 Latitude", Latitude);
    tb.sendTelemetryFloat("N0 Longitude", Longitude);
    tb.sendTelemetryFloat("N0 PDR", Pdr);
    tb.sendTelemetryFloat("N0 RTT", Rtt);
  } else if(Node_Id == 1) {
    tb.sendTelemetryFloat("N1 Speed", Speed);
    tb.sendTelemetryFloat("N1 Latitude", Latitude);
    tb.sendTelemetryFloat("N1 Longitude", Longitude);
    tb.sendTelemetryFloat("N1 PDR", Pdr);
    tb.sendTelemetryFloat("N1 RTT", Rtt);
  } else if(Node_Id == 2) {
    tb.sendTelemetryFloat("N2 Speed", Speed);
    tb.sendTelemetryFloat("N2 Latitude", Latitude);
    tb.sendTelemetryFloat("N2 Longitude", Longitude);
    tb.sendTelemetryFloat("N2 PDR", Pdr);
    tb.sendTelemetryFloat("N2 RTT", Rtt);
  }
}

// For Testing
float PRECEIVED[MAX_NODES];
float PSENT[MAX_NODES];
float PDR[MAX_NODES];

void setup() {
  // Setup GPS Module
  Serial.begin(9600);
  //MCU.begin(9600);

  // Setup LoRa Module
  /*
  LoRa.setPins(SS, RESET, DIO0);
  LoRa.setTxPower(TX_POWER);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);
  */
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RESET, DIO0);

  // Setup WiFi Module
  // InitWiFi();

  // Setup ThingsBoard Information
  // if(!tb.connected()) { Reconnect(); }

  // Tests Setup
  Serial.println("Starting Node " + String(NODE_ID) + ".");
  while(!LoRa.begin(433E6)) {
    Serial.println("Starting Node " + String(NODE_ID) + " Failed.");
    delay(10000);
  }
  Serial.println("Starting Node " + String(NODE_ID) + " Succeeded.");
}

void loop() {
  // if(!tb.connected()) { Reconnect(); }

  int PACKETSIZE = LoRa.parsePacket();
  if(PACKETSIZE) {
    byte PACKETBUFFER[50];
    LoRa.readBytes(PACKETBUFFER, PACKETSIZE);
           if(PACKETBUFFER[0] == DATA_PACKET && PACKETBUFFER[1] != NODE_ID && PACKETBUFFER[2] != NODE_ID  && ROUTINGTABLE[PACKETBUFFER[2]].isRouted == 1 && ROUTINGTABLE[PACKETBUFFER[2]].previousNode == PACKETBUFFER[1]) {
      // Receives DATA Packet
      float speed;
      memcpy(&speed, &PACKETBUFFER[3], sizeof(speed));
      float latitude;
      memcpy(&latitude, &PACKETBUFFER[7], sizeof(latitude));
      float longitude;
      memcpy(&longitude, &PACKETBUFFER[11], sizeof(longitude));
      float rtt;
      memcpy(&rtt, &PACKETBUFFER[16], sizeof(rtt));
      SendDATAPacket(DACK_PACKET, NODE_ID, PACKETBUFFER[2], speed, latitude, longitude, PACKETBUFFER[15], rtt);
      PRECEIVED[PACKETBUFFER[2]] += 1;
      PSENT[PACKETBUFFER[2]] = PACKETBUFFER[15];
      PDR[PACKETBUFFER[2]] = PRECEIVED[PACKETBUFFER[2]] / PSENT[PACKETBUFFER[2]];
      // UploadData(PACKETBUFFER[2], speed, latitude, longitude, PDR[PACKETBUFFER[2]], rtt);
      Serial.println("Received DATA Packet:");
      Serial.println("Packet Type:        " + String(PACKETBUFFER[0]));
      Serial.println("Intermediate Node:  " + String(PACKETBUFFER[1]));
      Serial.println("Source Node:        " + String(PACKETBUFFER[2]));    
      Serial.println("Speed:              " + String(speed, 6));
      Serial.println("Latitude:           " + String(latitude, 6));
      Serial.println("Longitude:          " + String(longitude, 6));
      Serial.println("Packets Received:   " + String(PRECEIVED[PACKETBUFFER[2]], 6));
      Serial.println("Packets Sent:       " + String(PSENT[PACKETBUFFER[2]], 6));
      Serial.println("PDR:                " + String(PDR[PACKETBUFFER[2]], 6));
      Serial.println("RTT:                " + String(rtt, 6));
      Serial.println("RSSI:               " + String(LoRa.packetRssi()));
    } else if(PACKETBUFFER[0] == RREQ_PACKET && PACKETBUFFER[1] < 25 && PACKETBUFFER[2] < MAX_NODES && PACKETBUFFER[3] != NODE_ID && PACKETBUFFER[4] != NODE_ID && PACKETBUFFER[5] != NODE_ID) {
      // Receives RREQ Packet and Sends RREP Packet
      SendAODVPacket(RREP_PACKET, PACKETBUFFER[1], PACKETBUFFER[2], PACKETBUFFER[5], PACKETBUFFER[4], NODE_ID);
      ROUTINGTABLE[PACKETBUFFER[4]] = {1, PACKETBUFFER[2], PACKETBUFFER[5], PACKETBUFFER[4], NODE_ID};
      Serial.println("Received RREQ Packet:");
      Serial.println("Packet Type:        " + String(PACKETBUFFER[0]));
      Serial.println("Broadcast Id:       " + String(PACKETBUFFER[1]));
      Serial.println("Hop Count:          " + String(PACKETBUFFER[2]));    
      Serial.println("Previous Node:      " + String(PACKETBUFFER[3]));
      Serial.println("Source Node:        " + String(PACKETBUFFER[4]));
      Serial.println("Next Node:          " + String(PACKETBUFFER[5]));
      Serial.println("RSSI:               " + String(LoRa.packetRssi()));
    }
  }

  // tb.loop();
}