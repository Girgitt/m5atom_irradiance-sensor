#ifndef CONFIG_H
#define CONFIG_H

#define USE_DAV6450  0                 // 0 = disable, 1 = read A0 pyranometer
#define USE_VEML7700  1
#define USE_WIFI  1
#define USE_MQTT  1

#define ROLLING_WINDOW_SIZE 25  // for 200ms update that gives 5s avg window
#define UPDATE_INTERVAL_MS 200

#define MQTT_PUBLISH_INTERVAL_MS 2000

// Wi-Fi credentials
#define WIFI_SSID "<your_wifi_ssid>"
#define WIFI_PASSWORD "<your_wifi_pass>"

// MQTT broker details
#define MQTT_SERVER "<your_mqtt_ip>>"
#define MQTT_PORT 1883
#define MQTT_USERNAME ""  // Optional, if required
#define MQTT_PASSWORD ""  // Optional, if required

// device ID for MQTT topics
#define MQTT_CLIENT_ID "irr_1"

// Define any other configuration parameters you want to keep secure
// #define ANOTHER_SETTING "value"

#endif // CONFIG_H