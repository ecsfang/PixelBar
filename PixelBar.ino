
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
#define N_PHASES  3

#define PULSE_TIME 250 // Heartbeat - 250ms = 1/4 sec

unsigned long blinkStart = 0; // the time the delay started
bool blinkRunning = false; // true if still waiting for delay to finish

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(N_PIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
typedef struct {
  float red;
  float green;
  float blue; 
} RGB_t;

float amp[N_PHASES];

typedef enum {
  NO_BLINK,
  BLINK,
  FAST_BLINK
} Blink_e;

Blink_e blink[N_PHASES];
int     blinkPeriod = 0;

// Current color of each phase ...
uint32_t  fColor[N_PHASES];

// Previous and current color of each led ...
uint32_t  oldColor[N_PIXELS];
uint32_t  newColor[N_PIXELS];

bool bUpdate = true;

void networkSetup(void)
{
  int f = 0;
  int dir = 1;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    pixels.setPixelColor(f, 0);
    pixels.setPixelColor(f+=dir, pixels.Color(0,0,255));
    if( f == (N_PIXELS-1) || f == 0 ) dir = -dir;
    pixels.show();
    if ((millis() - blinkStart) > 10000) {
      Serial.println("Connection Failed! Rebooting...");
      pixels.setPixelColor(f, 0);
      pixels.setPixelColor(3, pixels.Color(255,0,0));
      pixels.setPixelColor(4, pixels.Color(255,0,0));
      pixels.show();
      delay(1000);
      ESP.restart();
    }
  }
  Serial.println(WiFi.localIP());

  pixels.clear();
  pixels.show();
  
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  pixels.begin(); // This initializes the NeoPixel library.
  randomSeed(analogRead(0));

  for(int f=0; f<N_PHASES; f++) {
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

  networkSetup();

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

  if( n>=0 && n < N_PHASES ) {
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
  static int fass[N_PHASES] = {1,1,1};

  for(int f=0;f<N_PHASES;f++) {
    amp[f] += fass[f]*random(0,100*(f+1))/500.0;
    if( amp[f] > 30.0 ) fass[f] = -1;
    if( amp[f] < 0.0 ) fass[f] = 1;
  }
  bUpdate = true;
}

void loop() 
{
#ifdef USE_WIFI
  if ( WiFi.status() != WL_CONNECTED )
    networkSetup();

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

#define HALF_SECOND (500/PULSE_TIME)
#define SECOND      (1000/PULSE_TIME)

  if (blinkRunning && ((millis() - blinkStart) >= PULSE_TIME)) {
    blinkStart += PULSE_TIME; // this prevents drift in the delays

    for( int f=0; f<N_PHASES; f++) {
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
  int led = f*3; // Three leds for each phase, use the two first only ...
  switch( blink[f] ) {
    case NO_BLINK:
      setPixelColor(led+0, c);
      setPixelColor(led+1, c);
      break;
    case BLINK: // Normal blink 1 Hz ...
      setPixelColor(led+0, blinkPeriod & SECOND ? c : 0);
      setPixelColor(led+1, blinkPeriod & SECOND ? 0 : c);
      break;
    case FAST_BLINK: // Blink 2 Hz ...
      setPixelColor(led+0, blinkPeriod & HALF_SECOND ? c : 0);
      setPixelColor(led+1, blinkPeriod & HALF_SECOND ? c : 0);
      break;
  }
}

void updatePixels(void)
{
  // Check if any leds have been updated ...
  if( memcmp(oldColor, newColor, sizeof(uint32_t)*N_PIXELS) ) {
    pixels.show();
    memcpy(oldColor, newColor, sizeof(uint32_t)*N_PIXELS);
  }
  bUpdate = false;
}

#define SCALE_HI(x) (510.0*(x))     // 255 * 2.0 * x
#define SCALE_LO(x) (510.0*(1-(x))) // 255 * 2.0 * (1-x)

void phaseColor(int delayValue)
{
  RGB_t pixColor;
  float a, x;

  for( int f=0; f<N_PHASES; f++) {
    if( amp[f] > CRIT_VAL )
      blink[f] = FAST_BLINK;
    else if( amp[f] > WARN_VAL )
      blink[f] = BLINK;
    else
      blink[f] = NO_BLINK;
    
    if( amp[f] < MIN_VAL ) {
      // Low value - go from blue to green ...
      a = max(amp[f], 0);                 // 0 -> MIN
      x = a/MIN_VAL;                      // Value from 0 -> 1
      pixColor.red   = 0;
      pixColor.green = SCALE_HI(x);
      pixColor.blue  = SCALE_LO(x);
    } else {
      // High value - go from to red ...
      a = max(amp[f], MIN_VAL);           // MIN -> MAX
      a = min(a, MAX_VAL);                // MIN -> MAX
      x = (a-MIN_VAL)/(MAX_VAL-MIN_VAL);  // Value from 0 -> 1
      pixColor.red   = SCALE_HI(x);
      pixColor.green = SCALE_LO(x);
      pixColor.blue  = 0;
    }

    int r = min(int(pixColor.red),   255);
    int g = min(int(pixColor.green), 255);
    int b = min(int(pixColor.blue),  255);

    fColor[f] = pixels.Color(r,g,b);

    setPhaseColor(f, fColor[f]);
  }
  
  updatePixels();
  if( delayValue )
    delay(delayValue);
}
