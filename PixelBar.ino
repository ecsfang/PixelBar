
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

#include "mySSID.h"

#define USE_WIFI

#define PIN D8
#define N_PIXELS  8

unsigned long BLINK_TIME = 500; // 0.5 sec
unsigned long blinkStart = 0; // the time the delay started
bool blinkRunning = false; // true if still waiting for delay to finish

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
bool  blink[3];

int fas[3];
int fass[3];
uint32_t  fColor[3];

bool bUpdate = true;

void setup() 
{
  Serial.begin(115200);
  pixels.begin(); // This initializes the NeoPixel library.
  randomSeed(analogRead(0));
  memset(pixBar, 0, sizeof(RGB_t)*N_PIXELS);
  
  for(int f=0; f<3; f++) {
    fas[f] = 0;
    fass[f] = 1;
    blink[f] = false;
    fColor[f] = 0;
  }

  blinkStart = millis();
  blinkRunning = true;  

//  pixels.setBrightness(32);

  pixels.setPixelColor(0, pixels.Color(0,0,255));
  pixels.show();
 
#ifdef USE_WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    pixels.setPixelColor(0, pixels.Color(255,0,0));
    pixels.show();
    delay(5000);
    pixels.setPixelColor(0, pixels.Color(0,0,0));
    pixels.show();
    ESP.restart();
  }
  Serial.println(WiFi.localIP());

  pixels.setPixelColor(0, pixels.Color(0,255,0));
  pixels.show();
  
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
#endif
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

  if( strncmp(topic, "powermeter/phase", 16) != 0) {
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

void simulateAmp()
{
  for(int f=0;f<3;f++) {
    amp[f] += fass[f]*random(0,100*(f+1))/500.0;
    if( amp[f] > 25.0 ) fass[f] = -1;
    if( amp[f] < 0.0 ) fass[f] = 1;
  }
  bUpdate = true;
}

bool ledOn = false;

void loop() 
{
#ifdef USE_WIFI
  if (!client.connected())
    reconnect();

  if (client.connected()) {
    client.loop();
    ArduinoOTA.handle();
  }
#else
  simulateAmp();
#endif
  if( bUpdate )
    phaseColor(100);

  if (blinkRunning && ((millis() - blinkStart) >= BLINK_TIME)) {
    blinkStart += BLINK_TIME; // this prevents drift in the delays
    // toggle the led
    ledOn = !ledOn;
    for( int f=0; f<3; f++) {
      if( blink[f] ) {
        pixels.setPixelColor((f*3)+0, ledOn ? fColor[f] : 0);
        pixels.setPixelColor((f*3)+1, ledOn ? 0 : fColor[f]);
      }        
    }
    pixels.show();
  }
}
 
#define min(a,b) (a<b)?(a):(b)
#define max(a,b) (a>b)?(a):(b)

#define MIN_VAL 5.0   // 5 amps gives green light
#define MAX_VAL 20.0  // 20 amp gives red ...

void phaseColor(int delayValue)
{
  for( int f=0; f<3; f++) {
    float a = max(amp[f], MIN_VAL); // MIN -> MAX
    a = min(a, MAX_VAL); // MIN -> MAX
    float x = (a-MIN_VAL)/(MAX_VAL-MIN_VAL);  // Value from 0 -> 1

    blink[f] = amp[f] > MAX_VAL;
    
    // Make color from green to red (min -> max)
    pixBar[f].red   = 2.0 * x;
    pixBar[f].green = 2.0 * (1-x);

    int r = min(int(pixBar[f].red*255), 255);
    int g = min(int(pixBar[f].green*255), 255);

    fColor[f] = pixels.Color(r,g,0);

    if( blink[f] ) {
      pixels.setPixelColor((f*3)+0, ledOn ? fColor[f] : 0);
      pixels.setPixelColor((f*3)+1, ledOn ? 0 : fColor[f]);
    } else {
      pixels.setPixelColor((f*3)+0, fColor[f]);
      pixels.setPixelColor((f*3)+1, fColor[f]);
    }
  }
  
  pixels.show();
  bUpdate = false;
  delay(delayValue);
}
