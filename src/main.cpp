#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>


#include <PubSubClient.h>

#include "config.h"

Adafruit_VEML7700 veml;

#define ROLLING_WINDOW_SIZE 25  // for 200ms update that gives 5s avg window
#define UPDATE_INTERVAL_MS 200

#define MQTT_PUBLISH_INTERVAL_MS 2000

uint16_t rollingBuffer[ROLLING_WINDOW_SIZE] = {0};
uint8_t rollingIndex = 0;
uint8_t rollingCount = 0;

#if defined(ESP32)                     // ---------- M5Atom -------------
  #include <M5Atom.h>
  #include <WiFi.h>
  #define HAS_LEDS  1
  /* M5Atom I²C */
  constexpr int I2C_SDA = 25;
  constexpr int I2C_SCL = 21;

#elif defined(ESP8266)                 // ---------- D1 mini ------------
  #include <ESP8266WiFi.h> 
  #define HAS_LEDS  0

  constexpr int I2C_SDA = D2;
  constexpr int I2C_SCL = D1;

#else                                   // ---------- anything else ------
  #error "Supported boards: ESP32-based M5Atom or ESP8266-based Wemos D1 mini"
#endif


// WiFi credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT broker details
const char* mqtt_client_id = MQTT_CLIENT_ID; // Unique ID
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_username = MQTT_USERNAME;  // Optional, if required
const char* mqtt_password = MQTT_PASSWORD;  // Optional, if required
static unsigned long lastPublish = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// MQTT topic definitions
String global_status_topic = "sensors/" + String(mqtt_client_id) + "/status";
String lux_topic = "sensors/" + String(mqtt_client_id) + "/lux/";
String watt_topic = "sensors/" + String(mqtt_client_id) + "/watt";
String gain_topic = "sensors/" + String(mqtt_client_id) + "/gain";
String command_topic = "sensors/" + String(mqtt_client_id) + "/command";


/* ---------- LED helpers ------------------------------------------------ */
#if HAS_LEDS
  #include <FastLED.h>

    static inline uint8_t xyIdx(uint8_t x, uint8_t y) { return y * 5 + x; }

    /* two‑row bar (hundreds / tens). fills bottom cell first, then top */
    void drawBar2RowsScaled(uint8_t value, uint8_t row0, const CRGB &col, uint16_t scale)
    {
        // value: full value (e.g., val / 10 or val / 100)
        // scale: max input value for full bar (e.g., 10 for tens, 10 for hundreds)
        uint8_t filled = (value * 10 + scale / 2) / scale; // round to nearest of 10
        for (uint8_t p = 0; p < 10; ++p) {
            uint8_t x = p / 2;
            uint8_t y = row0 + ((p & 1) ? 0 : 1);  // even = bottom, odd = top
            M5.dis.drawpix(xyIdx(x, y), (p < filled) ? col : CRGB::Black);
        }
    }

    /* bottom row bar for ones: filled green, tip white (odd) or green (even) */
    void drawOnes(uint8_t digit)
    {
        uint8_t filled = (digit * 5 + 4) / 9;      // 0‑9 → 0‑5 pixels
        for (uint8_t x = 0; x < 5; ++x)
        {
            if (x < filled)
            {
                bool tip   = (x == filled - 1);
                CRGB color = tip ? (digit & 1 ? CRGB::White : CRGB::Green)
                                : CRGB(0, 64, 0);           // dim green body
                M5.dis.drawpix(xyIdx(x, 4), color);
            }
            else
                M5.dis.drawpix(xyIdx(x, 4), CRGB::Black);
        }
    }

    void showValue(uint16_t v)
    {
        M5.dis.clear();
        drawBar2RowsScaled(v / 100, 0, CRGB::Red, 10);           // hundreds, 0–999 → 0–10
        drawBar2RowsScaled((v / 10) % 10, 2, CRGB::Yellow, 10);  // tens, 0–99 → 0–10
        drawOnes(v % 10);                                        // row 4
    }
#else
    /* ------ Stub versions so the rest of the sketch compiles identically */
    inline void showValue(uint16_t) {}
#endif  /* HAS_LEDS */

/* ---------- simple auto‑range: gain + integration time ----------------- */
void setRange(uint8_t gIdx, uint8_t itIdx)
{
    switch (gIdx) {
      case 0: veml.setGain(VEML7700_GAIN_1_8); break;
      case 1: veml.setGain(VEML7700_GAIN_1_4); break;
      case 2: veml.setGain(VEML7700_GAIN_1  ); break;
      case 3: veml.setGain(VEML7700_GAIN_2  ); break;
    }
    switch (itIdx) {
      case 0: veml.setIntegrationTime(VEML7700_IT_25MS ); break;
      case 1: veml.setIntegrationTime(VEML7700_IT_50MS ); break;
      case 2: veml.setIntegrationTime(VEML7700_IT_100MS); break;
      case 3: veml.setIntegrationTime(VEML7700_IT_200MS); break;
    }
}

void setup_wifi() {
  delay(10);
  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(" connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Callback function to handle received messages
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("handling command message: "+message);
  // Check if the received message is for servo
  if (String(topic) == String(command_topic)) {
    
    Serial.println("handling command: "+message);
    if (message == "something") {
      ;// do something
    }
  }
}

// Reconnect to MQTT server
void reconnect() {
  // Loop until we are connected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected to mqtt broker");
      
      // Subscribe to the command topic
      client.subscribe(command_topic.c_str()); // Convert String to const char* using c_str()

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/* --------------------------- setup ------------------------------------ */
void setup()
{
#if HAS_LEDS
    M5.begin(true, false, true);          // LEDs, no serial over USB-C
#else
    Serial.begin(115200);                 // Serial on Wemos
#endif
    
Wire.begin(I2C_SDA, I2C_SCL);
if (!veml.begin()) {
    Serial.println(F("VEML7700 not found"));
    while (true) delay(100);
}

setRange(0, 0);                       // gain 1/8, IT 25 ms (sunlight)

setup_wifi();

// Set up MQTT client
client.setServer(mqtt_server, mqtt_port);
client.setCallback(callback);

// Connect to MQTT server
reconnect();
if (client.connected()){
    Serial.println(F("connected to MQTT broker"));
}

client.setKeepAlive(60);

#if HAS_LEDS
    M5.dis.clear();
#endif
}

/* ---------------------------- loop ------------------------------------ */
void loop()
{
    static uint8_t g = 0, it = 0;           // current gain / IT indices
    //float lux = veml.readLux();
    float lux = veml.readLuxNormalized();  // correction for high-lux setRange(0, 0) which is gain 1/8, IT 25 ms
    

    // /* auto‑range: keep lux in ~300‑25 000 */
    //if (lux > 25000 && g > 0) { --g; setRange(g, it); return; }
    //if (lux <   300 && g < 3) { ++g; setRange(g, it); return; }

    //float wm2 = lux / 126.7f;               // daylight luminous efficacy - theoretical
    float wm2 = lux / 122;                    // based on https://www.extrica.com/article/21667
    
    

    // rolling average for wm2
    rollingBuffer[rollingIndex] = wm2;
    rollingIndex = (rollingIndex + 1) % ROLLING_WINDOW_SIZE;
    if (rollingCount < ROLLING_WINDOW_SIZE) rollingCount++;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < rollingCount; ++i) sum += rollingBuffer[i];
    uint16_t wm2_avgVal = (sum + rollingCount / 2) / rollingCount;

    // capping wm2 value to 1-999 for LED bargraph display
    uint16_t val_max_999 = wm2_avgVal > 999 ? 999 : (uint16_t)round(wm2_avgVal);
    uint16_t wm2_avg_val__min_1__max_999 = val_max_999 < 1 ? 1 : val_max_999;

    Serial.printf("Lux:%7.0f  |  W/m² (1-999):%4u  |  W/m² raw: %4u |  gain: %4u \n", lux, wm2_avg_val__min_1__max_999, wm2_avgVal, g);
    showValue(wm2_avg_val__min_1__max_999);

    if (client.connected()){
        client.loop(); // Process incoming messages
        
        unsigned long now = millis();

        if (now - lastPublish >= MQTT_PUBLISH_INTERVAL_MS) {
            lastPublish = now;
            
            char payload[10];

            dtostrf(lux, 1, 2, payload);
            client.publish(lux_topic.c_str(), payload, 1);

            dtostrf(wm2_avgVal, 1, 2, payload);
            client.publish(watt_topic.c_str(), payload, 1);

            dtostrf(g, 1, 2, payload);
            client.publish(gain_topic.c_str(), payload, 1);
        }

    }
    else{
        reconnect();
    }

    delay(UPDATE_INTERVAL_MS);
}
