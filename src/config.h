#ifndef CONFIG_H
#define CONFIG_H

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