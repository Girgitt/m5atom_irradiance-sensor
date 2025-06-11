#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>


#include <PubSubClient.h>

#include "config.h"

// ---- default USE flags ----
#ifndef USE_DAV6450
#define USE_DAV6450 0
#endif

#ifndef USE_VEML7700
#define USE_VEML7700 1
#endif

#ifndef USE_WIFI
#define USE_WIFI 0
#endif

#ifndef USE_MQTT
#define USE_MQTT 0
#endif


// ---- Default timing settings ----
#ifndef ROLLING_WINDOW_SIZE
#define ROLLING_WINDOW_SIZE 25
#endif

#ifndef UPDATE_INTERVAL_MS
#define UPDATE_INTERVAL_MS 200
#endif

#ifndef MQTT_PUBLISH_INTERVAL_MS
#define MQTT_PUBLISH_INTERVAL_MS 2000
#endif

// ---------- Wi-Fi / MQTT availability parameters ------------------
#define WIFI_MAX_SETUP_ATTEMPTS      10     // once, during setup()
#define WIFI_RETRY_INTERVAL_MS    10000UL   // 10 s between retries in loop()
#define WIFI_MAX_LOOP_ATTEMPTS       15     // before we reboot

#define MQTT_RETRY_INTERVAL_MS    10000UL
#define MQTT_MAX_LOOP_ATTEMPTS       20

Adafruit_VEML7700 veml;

uint16_t rollingBuffer[ROLLING_WINDOW_SIZE] = {0};
uint16_t rollingBufferHi[ROLLING_WINDOW_SIZE] = {0};
uint8_t rollingIndex = 0;
uint8_t rollingCount = 0;


#if USE_DAV6450
  #if defined(ESP32)                  // ----- M5-Atom -----------------
    constexpr int DAV_PIN       = 36; // GPIO36 = ADC1_CH0 on Atom
    constexpr int ADC_COUNTS    = 4095;
    constexpr int ADC_VREF_mV   = 3300;
  #elif defined(ESP8266)              // ----- Wemos D1 mini ----------
    constexpr int DAV_PIN       = A0; // built-in divider: 0-3.3 V → 0-1.0 V
    constexpr int ADC_COUNTS    = 1023;
    constexpr int ADC_VREF_mV   = 3300;
  #endif
#endif

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
String lux_topic = "sensors/" + String(mqtt_client_id) + "/lux";
String lux_hi_topic = "sensors/" + String(mqtt_client_id) + "/lux_hi";
String watt_topic = "sensors/" + String(mqtt_client_id) + "/watt";
String watt_hi_topic = "sensors/" + String(mqtt_client_id) + "/watt_hi";
String watt_dav_topic = "sensors/" + String(mqtt_client_id) + "/watt_dav";
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

bool setupWifiOnce()
{
  Serial.print(F("Connecting to Wi-Fi"));
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);          // we’ll manage it ourselves
  WiFi.begin(ssid, password);

  for (int i = 0; i < WIFI_MAX_SETUP_ATTEMPTS; ++i)
  {
    if (WiFi.waitForConnectResult(3000) == WL_CONNECTED)
    {
      Serial.print(F("  ✔  IP="));
      Serial.println(WiFi.localIP());
      return true;
    }
    Serial.print('.');
  }
  Serial.println(F("  ✖  giving up for now"));
  return false;                          // let loop() take over
}

/* ----------- keepWifiAlive()  – non-blocking Wi-Fi watchdog ----------- */
bool keepWifiAlive()
{
  static uint32_t lastTry   = 0;
  static uint8_t  attempts  = 0;

  if (WiFi.status() == WL_CONNECTED) { attempts = 0; return true; }

  uint32_t now = millis();
  if (now - lastTry < WIFI_RETRY_INTERVAL_MS) return false;   // wait

  lastTry = now;
  ++attempts;
  Serial.printf("\n[Wi-Fi] reconnect attempt %u/%u …\n",
                attempts, WIFI_MAX_LOOP_ATTEMPTS);

  WiFi.disconnect(true);                 // start from a clean slate
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult(5000) == WL_CONNECTED)
  {
    Serial.println(F("[Wi-Fi] reconnected."));
    attempts = 0;
    return true;
  }

  if (attempts >= WIFI_MAX_LOOP_ATTEMPTS)
  {
    Serial.println(F("[Wi-Fi] giving up – rebooting"));
    delay(200);
    ESP.restart();
  }
  return false;
}

/* ------ keepMqttAlive()  – non-blocking MQTT watchdog -------------- */
void keepMqttAlive()
{
  static uint32_t lastTry   = 0;
  static uint8_t  attempts  = 0;

  if (client.connected()) { attempts = 0; client.loop(); return; }
  if (WiFi.status() != WL_CONNECTED)     { return; }          // Wi-Fi first

  uint32_t now = millis();
  if (now - lastTry < MQTT_RETRY_INTERVAL_MS) return;         // wait

  lastTry = now;
  ++attempts;
  Serial.printf("[MQTT] reconnect attempt %u/%u …\n",
                attempts, MQTT_MAX_LOOP_ATTEMPTS);

  if (client.connect(mqtt_client_id, mqtt_username, mqtt_password))
  {
    Serial.println(F("[MQTT] connected."));
    client.subscribe(command_topic.c_str());
    attempts = 0;
  }
  else
  {
    Serial.printf("[MQTT] rc=%d\n", client.state());
    if (attempts >= MQTT_MAX_LOOP_ATTEMPTS)
    {
      Serial.println(F("[MQTT] giving up – rebooting"));
      delay(200);
      ESP.restart();
    }
  }
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

/* Return a reading scaled to 0-2047 (≈ 11 effective bits)            */
/* We take 4 raw samples (10-bit), sum them, right-shift one bit.      */
uint16_t analogReadOversampled11(uint8_t pin)
{
    uint32_t sum = 0;
    for (int i = 0; i < 4; ++i) {
        
        unsigned long loop_time = micros();
        while(micros()-loop_time<150){
          yield();
        }
        //wifi_set_opmode(NULL_MODE);
        
        // with disabled soft wdt wifi gets interrupted
        // #if defined(ESP8266) 
        //   system_soft_wdt_stop();
        // #endif
        ets_intr_lock( ); 
        noInterrupts();

        sum += analogRead(pin);   // 0…1023 each
        
        interrupts();
        ets_intr_unlock(); 
        // #if defined(ESP8266) 
        //   system_soft_wdt_restart();
        // #endif
        //delayMicroseconds(150);   // changed to yield before reading
    }
    return ((sum + 2) >> 1);        // divide by 2 with rounding
}

/* ------------------------------------------------------------------
 *  Smoothed ADC read: 10 samples  →  drop max & min  →  average 8
 * ------------------------------------------------------------------ */
uint16_t analogReadTrimmedAvg(uint8_t pin)
{
    uint32_t sum   = 0;
    uint16_t v_min = 0xFFFF, v_max = 0;

    for (uint8_t i = 0; i < 10; ++i) {
        //uint16_t v = analogRead(pin);
        uint16_t v = analogReadOversampled11(pin);  // oversampling returns up to 2047
        sum += v;
        if (v < v_min) v_min = v;
        if (v > v_max) v_max = v;
        //delay(2);                       // 1 ms between samples
    }

    sum -= (v_min + v_max);             // drop extremes
    return static_cast<uint16_t>(sum / 8 / 2);   // average of the remaining 8 divided additionally by 2 due to oversampling
}

/* --------------------------- setup ------------------------------------ */
void setup()
{
#if HAS_LEDS
    M5.begin(true, false, true);          // LEDs, no serial over USB-C
#else
    Serial.begin(115200);                 // Serial on Wemos
#endif

#if USE_DAV6450 && defined(ESP32)
  analogReadResolution(12);               // 0-4095
  analogSetAttenuation(ADC_11db);         // 0-3.3 V span
#endif

#if USE_VEML7700
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!veml.begin()) {
      Serial.println(F("VEML7700 not found"));
      while (true) delay(100);
  }
  setRange(0, 0);                       // gain 1/8, IT 25 ms (sunlight)
#endif


#if USE_WIFI
  setupWifiOnce();
#endif

#if USE_MQTT
  // Set up MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // reconnection is moved to main loop
  //reconnect();
  //if (client.connected()){
  //    Serial.println(F("connected to MQTT broker"));
  //}

  client.setKeepAlive(60);
#endif

#if HAS_LEDS
    M5.dis.clear();
#endif
}

/* ---------------------------- loop ------------------------------------ */
void loop()
{

  #if USE_WIFI
    if (!keepWifiAlive())                  // returns immediately
    {
      delay(UPDATE_INTERVAL_MS);           // still give CPU a short break
      return;                              // skip the rest of the loop
    }
  #endif

  #if USE_WIFI && USE_MQTT
    keepMqttAlive();                       // only runs when Wi-Fi OK
  #endif

  static uint8_t g = 0, it = 0;           // current gain / IT indices
  #if USE_VEML7700
    //float lux = veml.readLux();
    float lux_hi = veml.readLuxNormalized();  // correction for high-lux setRange(0, 0) which is gain 1/8, IT 25 ms
    float lux = veml.readLux();

    // /* auto‑range: keep lux in ~300‑25 000 */
    //if (lux > 25000 && g > 0) { --g; setRange(g, it); return; }
    //if (lux <   300 && g < 3) { ++g; setRange(g, it); return; }

    //float wm2 = lux / 126.7f;               // daylight luminous efficacy - theoretical
    float wm2 = lux / 122;                    // based on https://www.extrica.com/article/21667
    float wm2_hi = lux_hi / 122; 

  #else
    float lux = -1;
    float lux_hi = -1;
    float wm2 = -1;
    float wm2_hi = -1;
  #endif
  
  #if USE_DAV6450
    int    raw      = analogReadTrimmedAvg(DAV_PIN);
    float  mV       = ((float)raw / ADC_COUNTS) * ADC_VREF_mV;
    float  dav_wm2  = mV / 1.67f;           // 1.67 mV → 1 W m⁻²
  #else
    float  dav_wm2  = -1;
  #endif

      // rolling average for wm2
    rollingBuffer[rollingIndex] = wm2;
    rollingBufferHi[rollingIndex] = wm2_hi;

    rollingIndex = (rollingIndex + 1) % ROLLING_WINDOW_SIZE;
    if (rollingCount < ROLLING_WINDOW_SIZE) rollingCount++;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < rollingCount; ++i) sum += rollingBuffer[i];
    uint16_t wm2_avgVal = (sum + rollingCount / 2) / rollingCount;

    sum = 0;
    for (uint8_t i = 0; i < rollingCount; ++i) sum += rollingBufferHi[i];
    uint16_t wm2_hi_avgVal = (sum + rollingCount / 2) / rollingCount;

    // capping wm2 value to 1-999 for LED bargraph display
    uint16_t val_max_999 = wm2_avgVal > 999 ? 999 : (uint16_t)round(wm2_avgVal);
    uint16_t wm2_avg_val__min_1__max_999 = val_max_999 < 1 ? 1 : val_max_999;

    Serial.printf("Lux:%7.0f  | Lux Hi:%7.0f  | W/m² (1-999):%4u  |  W/m²: %4u | W/m² Hi: %4u | gain: %4u | W/m² DAV: %5.0f \n", lux, lux_hi, wm2_avg_val__min_1__max_999, wm2_avgVal, wm2_hi_avgVal, g, dav_wm2);
  
  showValue(wm2_avg_val__min_1__max_999);


  #if USE_WIFI && USE_MQTT
    if (client.connected()){
        client.loop(); // Process incoming messages
        
        unsigned long now = millis();

        if (now - lastPublish >= MQTT_PUBLISH_INTERVAL_MS) {
            lastPublish = now;
            
            char payload[10];
            #if USE_VEML7700
              dtostrf(lux, 1, 2, payload);
              client.publish(lux_topic.c_str(), payload, 1);

              dtostrf(lux_hi, 1, 2, payload);
              client.publish(lux_hi_topic.c_str(), payload, 1);

              dtostrf(wm2_avgVal, 1, 2, payload);
              client.publish(watt_topic.c_str(), payload, 1);

              dtostrf(wm2_hi_avgVal, 1, 2, payload);
              client.publish(watt_hi_topic.c_str(), payload, 1);

              dtostrf(g, 1, 2, payload);
              client.publish(gain_topic.c_str(), payload, 1);
            #endif

            #if USE_DAV6450
              dtostrf(dav_wm2, 1, 2, payload);
              client.publish(watt_dav_topic.c_str(), payload, 1);

            #endif

        }

    }

  #endif

    delay(UPDATE_INTERVAL_MS);
}
