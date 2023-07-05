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
#define SS    10
#define RESET 9
#define DIO0  2
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
  uint8_t Psent;
  float Trtt;
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
uint8_t dataCounter = 0;

// Sends DATA Packets
void sendDataPacket(uint8_t packetType, uint8_t intermediateNode, uint8_t sourceNode, float speed, float latitude, float longitude, uint8_t Psent, float Trtt) {
  dataPacket DataPacket = {packetType, intermediateNode, sourceNode, speed, latitude, longitude, Psent, Trtt};
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
void reconnect() {
  while(!tb.connected()) {
    status = WiFi.status();
    if(status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if(tb.connect(thingsboardServer, TOKEN)) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      delay( 5000 );
    }
  }
}

// Sends Data to ThingsBoard
void sendData(uint8_t node_id, float speed, float latitude, float longitude, float pdr) {
         if(node_id == 1) {
    tb.sendTelemetryFloat("N1 Speed", speed);
    tb.sendTelemetryFloat("N1 Latitude", latitude);
    tb.sendTelemetryFloat("N1 Longitude", longitude);
    tb.sendTelemetryFloat("N1 PDR", pdr);
  } else if(node_id == 2) {
    tb.sendTelemetryFloat("N2 Speed", speed);
    tb.sendTelemetryFloat("N2 Latitude", latitude);
    tb.sendTelemetryFloat("N2 Longitude", longitude);
    tb.sendTelemetryFloat("N2 PDR", pdr);
  } else if(node_id == 3) {
    tb.sendTelemetryFloat("N3 Speed", speed);
    tb.sendTelemetryFloat("N3 Latitude", latitude);
    tb.sendTelemetryFloat("N3 Longitude", longitude);
    tb.sendTelemetryFloat("N3 PDR", pdr);
  }
}

// For Testing
uint8_t Preceived[MAX_NODES];
uint8_t Psent[MAX_NODES];
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

  // Setup WiFi Module
  InitWiFi();

  // Setup ThingsBoard Information
  if(!tb.connected()) { reconnect(); }

  // Tests Setup
  Serial.println("Starting Node " + String(NODE_ID) + ".");
  while(!LoRa.begin(433E6)) {
    Serial.println("Starting Node " + String(NODE_ID) + " Failed.");
    delay(10000);
  }
  Serial.println("Starting Node " + String(NODE_ID) + " Succeeded");
}

void loop() {
  if(!tb.connected()) { reconnect(); }

  int packetSize = LoRa.parsePacket();
  if(packetSize) {
    byte packetBuffer[50];
    LoRa.readBytes(packetBuffer, packetSize);
           if(packetBuffer[0] == DATA_PACKET && packetBuffer[1] != NODE_ID && packetBuffer[2] != NODE_ID  && RoutingTable[packetBuffer[2]].isRouted == 1 && RoutingTable[packetBuffer[2]].previousNode == packetBuffer[1]) {
      // Receives DATA Packet
      float speed;
      memcpy(&speed, &packetBuffer[3], sizeof(speed));
      float latitude;
      memcpy(&latitude, &packetBuffer[7], sizeof(latitude));
      float longitude;
      memcpy(&longitude, &packetBuffer[11], sizeof(longitude));
      float trtt;
      memcpy(&trtt, &packetBuffer[16], sizeof(trtt));
      sendDataPacket(DACK_PACKET, NODE_ID, packetBuffer[2], speed, latitude, longitude, packetBuffer[15], trtt);
      Preceived[packetBuffer[2]] += 1;
      Psent[packetBuffer[2]] = packetBuffer[15];
      PDR[packetBuffer[2]] = Preceived[packetBuffer[2]] / Psent[packetBuffer[2]];
      sendData(packetBuffer[2], speed, latitude, longitude, PDR[packetBuffer[2]]);
      Serial.println("Received DATA Packet:");
      Serial.println("Packet Type:        " + String(packetBuffer[0]));
      Serial.println("Intermediate Node:  " + String(packetBuffer[1]));
      Serial.println("Source Node:        " + String(packetBuffer[2]));    
      Serial.println("Speed:              " + String(speed, 10));
      Serial.println("Latitude:           " + String(latitude, 10));
      Serial.println("Longitude:          " + String(longitude, 10));
      Serial.println("Packets Received:   " + String(Preceived[packetBuffer[2]]));
      Serial.println("Packets Sent:       " + String(Psent[packetBuffer[2]]));
      Serial.println("PDR:                " + String(PDR[packetBuffer[2]]));
      Serial.println("RSSI:               " + String(LoRa.packetRssi()));
    } else if(packetBuffer[0] == RREQ_PACKET && packetBuffer[1] < 255 && packetBuffer[2] < MAX_NODES && packetBuffer[3] != NODE_ID && packetBuffer[4] != NODE_ID && packetBuffer[5] != NODE_ID) {
      // Receives RREQ Packet and Sends RREP Packet
      sendAODVPacket(RREP_PACKET, packetBuffer[1], packetBuffer[2], packetBuffer[5], packetBuffer[4], NODE_ID);
      RoutingTable[packetBuffer[4]] = {1, packetBuffer[2], packetBuffer[5], packetBuffer[4], NODE_ID};
      Serial.println("Received RREQ Packet:");
      Serial.println("Packet Type:        " + String(packetBuffer[0]));
      Serial.println("Broadcast Id:       " + String(packetBuffer[1]));
      Serial.println("Hop Count:          " + String(packetBuffer[2]));    
      Serial.println("Previous Node:      " + String(packetBuffer[3]));
      Serial.println("Source Node:        " + String(packetBuffer[4]));
      Serial.println("Next Node:          " + String(packetBuffer[5]));
      Serial.println("RSSI:               " + String(LoRa.packetRssi()));
    }
  }

  tb.loop();
}