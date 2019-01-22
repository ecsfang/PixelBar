
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

unsigned long BLINK_TIME = 250; // 1/4 sec
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

typedef enum {
  NO_BLINK,
  BLINK,
  FAST_BLINK
} Blink_e;

Blink_e blink[3];
int     blinkPeriod = 0;

int fas[3];
int fass[3];
uint32_t  fColor[3];

uint32_t  oldColor[N_PIXELS];
uint32_t  newColor[N_PIXELS];

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
    blink[f] = NO_BLINK;
    fColor[f] = 0;
  }

  blinkStart = millis();
  blinkRunning = true;  

  pixels.setBrightness(32);

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
    if( amp[f] > 30.0 ) fass[f] = -1;
    if( amp[f] < 0.0 ) fass[f] = 1;
  }
  bUpdate = true;
}

void loop() 
{
#ifdef USE_WIFI
  if (!client.connected())
    reconnect();

  if (client.connected()) {
    client.loop();
    ArduinoOTA.handle();
  }
  if( bUpdate )
    phaseColor(0);
#else
  simulateAmp();
  phaseColor(100);
#endif

#define HALF_SECOND 0x02
#define SECOND      0x04

  if (blinkRunning && ((millis() - blinkStart) >= BLINK_TIME)) {
    blinkStart += BLINK_TIME; // this prevents drift in the delays

    for( int f=0; f<3; f++) {
      // Check if any led should blink ... then do so!
      if( blink[f] != NO_BLINK ) {
        setPhaseColor(f, fColor[f]);
        bUpdate = true;
      }
    }
    if( bUpdate ) {
      pixels.show();
      bUpdate = false;
    }
    blinkPeriod++;
  }
}
 
#define min(a,b) (a<b)?(a):(b)
#define max(a,b) (a>b)?(a):(b)

#define MIN_VAL 5.0   // 5 amps gives green light
#define MAX_VAL 25.0  // 25 amp gives red ...
#define WARN_VAL 20.0 // 20.0 amp gives blinking red ...
#define CRIT_VAL 22.5 // 22.5 amp gives fast blinking red ...

void setPixelColor(int l, uint32_t c)
{
  newColor[l] = c;
  pixels.setPixelColor(l, c);
}

void setPhaseColor(int f, uint32_t c)
{
  switch( blink[f] ) {
    case NO_BLINK:
      setPixelColor((f*3)+0, c);
      setPixelColor((f*3)+1, c);
      break;
    case BLINK: // Normal blink 1 Hz ...
      setPixelColor((f*3)+0, blinkPeriod & SECOND ? c : 0);
      setPixelColor((f*3)+1, blinkPeriod & SECOND ? 0 : c);
      break;
    case FAST_BLINK: // Blink 2 Hz ...
      setPixelColor((f*3)+0, blinkPeriod & HALF_SECOND ? c : 0);
      setPixelColor((f*3)+1, blinkPeriod & HALF_SECOND ? c : 0);
      break;
  }
}

void updatePixels(void)
{
  if( memcmp(oldColor, newColor, sizeof(uint32_t)*N_PIXELS) ) {
    pixels.show();
    memcpy(oldColor, newColor, sizeof(uint32_t)*N_PIXELS);
  }
  bUpdate = false;
}

void phaseColor(int delayValue)
{
  for( int f=0; f<3; f++) {
    float a = max(amp[f], MIN_VAL); // MIN -> MAX
    a = min(a, MAX_VAL); // MIN -> MAX
    float x = (a-MIN_VAL)/(MAX_VAL-MIN_VAL);  // Value from 0 -> 1

    blink[f] = NO_BLINK;
    if( amp[f] > WARN_VAL ) blink[f] = BLINK;
    if( amp[f] > CRIT_VAL ) blink[f] = FAST_BLINK;
    
    // Make color from green to red (min -> max)
    if( amp[f] < MIN_VAL ) {
      a = max(amp[f], 0); // 0 -> MIN
      x = a/MIN_VAL;  // Value from 0 -> 1
      pixBar[f].red   = 0;
      pixBar[f].green = 2.0 * x;
      pixBar[f].blue  = 2.0 * (1-x);
    } else {
      pixBar[f].red   = 2.0 * x;
      pixBar[f].green = 2.0 * (1-x);
      pixBar[f].blue  = 0;
    }

    int r = min(int(pixBar[f].red*255), 255);
    int g = min(int(pixBar[f].green*255), 255);
    int b = min(int(pixBar[f].blue*255), 255);

    fColor[f] = pixels.Color(r,g,b);

    setPhaseColor(f, fColor[f]);
  }
  
  updatePixels();
  if( delayValue )
    delay(delayValue);
}
