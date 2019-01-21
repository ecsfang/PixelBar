
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

#include "mySSID.h"

#define PIN D8
#define N_PIXELS  8

WiFiClient espClient;
PubSubClient client(espClient);

//the Wemos WS2812B RGB shield has 1 LED connected to pin 2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);
 
typedef struct {
  float red;
  float green;
  float blue; 
} RGB_t;

RGB_t pixBar[N_PIXELS];

float amp[3];

int fas[3];
int fass[3];

bool bUpdate = true;

void setup() 
{
  pixels.begin(); // This initializes the NeoPixel library.
  randomSeed(analogRead(0));
  memset(pixBar, 0, sizeof(RGB_t)*N_PIXELS);
  fas[0] = fas[1] = fas[2] = 0;
  fass[0] = fass[1] = fass[2] = 1;
  pixels.setBrightness(32);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println(WiFi.localIP());

  Serial.println();

  if( WiFi.status() == WL_CONNECTED ) {

    ArduinoOTA.setHostname("PhaseDisplay");
    ArduinoOTA.setPassword(flashpw);
  
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      } else {
        Serial.println("Unknown error!");
      }
    });
    ArduinoOTA.begin();
  }
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void callback(char* topic, byte* payload, unsigned int length) {
  char msg[32+1];

  strncpy(msg,(char*)payload,32);
  
#ifdef RX_DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msg);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  if( strncmp(topic, "powermeter/phase",16) != 0) {
    return;
  }
  
  //powermeter/phaseX
  //01234567890123456
  int n = topic[16]-'1';

  if( n>=0 && n < 3 ) {
    String sTemp = String((char*)payload);
    amp[n] = sTemp.toFloat();
    bUpdate = true;
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting another MQTT connection...");
    // Attempt to connect
    if (client.connect("thePhaseDisplay")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("phaseDisplay", "ready");
      // ... and resubscribe
      client.subscribe("powermeter/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() 
{
  if (!client.connected())
    reconnect();

  if (client.connected()) {
    client.loop();
    ArduinoOTA.handle();
  }

  if( bUpdate )
    phaseColor(100);
}
 
#define min(a,b) (a<b)?(a):(b)
#define max(a,b) (a>b)?(a):(b)

#define MAX_VAL 20.0

void phaseColor(int delayValue)
{
  for( int f=0; f<3; f++) {
    float x = min(amp[f]/MAX_VAL, 1);  // Value from 0 -> 1
    pixBar[f].red   = 2.0 * x;
    pixBar[f].green = 2.0 * (1-x);

    int r = min(max(int(pixBar[f].red*255),0), 255);
    int g = min(max(int(pixBar[f].green*255),0), 255);
 
    pixels.setPixelColor((f*3)+0, pixels.Color(r,g,0));
    pixels.setPixelColor((f*3)+1, pixels.Color(r,g,0));
  }
  
  pixels.show();
  bUpdate = false;
  delay(delayValue);
}
