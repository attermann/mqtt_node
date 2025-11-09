
// Macro to read build flags
#define ST(A) #A
#define STR(A) ST(A)

#ifdef WIFI_SSID
const char *ssid = STR(WIFI_SSID);
#endif

#ifdef WIFI_PASSWORD
const char *password = STR(WIFI_PASSWORD);
#endif

#ifdef MQTT_SERVER
const char *mqtt_server = STR(MQTT_SERVER);
#endif

#ifdef MQTT_PORT
const int mqtt_port = atoi(STR(MQTT_PORT));
#endif

#ifdef MQTT_USERNAME
const char *mqtt_username = STR(MQTT_USERNAME);
#endif

#ifdef MQTT_PASSWORD
const char *mqtt_password = STR(MQTT_PASSWORD);
#endif

#ifdef MQTT_ROOT
const char *mqtt_root = STR(MQTT_ROOT);
#endif

#ifdef DEVICE_NAME
const char *device_name = STR(DEVICE_NAME);
#endif
