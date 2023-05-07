
#include <WiFi.h>
#include <ThingsBoard.h>

#define WIFI_AP "GTE 2.4g"
#define WIFI_PASSWORD "gte646464"

#define TOKEN "GATEWAY_TOKEN"

char thingsboardServer[] = "thingsboard.cloud";

WiFiClient wifiClient;

ThingsBoard tb(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

// Setup Data Packet Structure
struct dataPacket {
  int sourceNode;
  double latitude;
  double longitude;
  double groundSpeed;
};

void setup()
{
  Serial.begin(115200);
  delay(10);
  InitWiFi();
  lastSend = 0;
  


}

void loop()
{
 

  if ( !tb.connected() ) {
    reconnect();
  }
  
  struct dataPacket d1;

  d1.sourceNode = 0;
  d1.latitude = 14.645505;
  d1.longitude = 121.061435;
  d1.groundSpeed = 2.22;

  if ( millis() - lastSend > 10000 ) { // Update and send only after 10 seconds
    SendDummyData(d1);
    lastSend = millis();

  }

  tb.loop();
}

void SendDummyData(struct dataPacket p)
{
  Serial.println("Collecting data.");


  Serial.println("Sending data to ThingsBoard:");
  Serial.print("Jeep ID: ");
  Serial.println(p.sourceNode);
  Serial.print("Latitude: ");
  Serial.println(p.latitude);
  Serial.print("Longitude: ");
  Serial.println(p.longitude);
  Serial.print("Ground Speed: ");
  Serial.println(p.groundSpeed);


  tb.sendTelemetryFloat("Jeep ID", p.sourceNode);
  tb.sendTelemetryFloat("latitude", p.latitude);
  tb.sendTelemetryFloat("longitude", p.longitude);
  tb.sendTelemetryFloat("Ground Speed", p.groundSpeed);
}


void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}