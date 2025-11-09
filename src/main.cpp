
/**
 * 
 * State topics:
 *     /device/<device_name>/state/temperature
 *     /device/<device_name>/state/relay
 *     /device/<device_name>/state/setpoint
 *     /device/<device_name>/state/settings
 * 
 * Control topics:
 *     /device/<device_name>/control/setpoint
 *     /device/<device_name>/control/settings
 * 
 */

#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <max6675.h>
#include <time.h>

#include "env.h"

//#define DEFAULT_TEMP_SETPOINT 37.0f      // Temperature setpoint
#define DEFAULT_TEMP_SETPOINT 2.78f      // Temperature setpoint
//#define DEFAULT_TEMP_DIFFERENTIAL 3.0f   // Temperature differential
#define DEFAULT_TEMP_DIFFERENTIAL 1.67f   // Temperature differential
//#define DEFAULT_TEMP_CORRECTION -4.5f    // Correction to apply to measurements read from sensor
#define DEFAULT_TEMP_CORRECTION -2.5f    // Correction to apply to measurements read from sensor
#define DEFAULT_INTERVAL 10000

//#define RELAY_UPDATE_INTERVAL 60000 // Number of milliseconds between relay updates
#define RELAY_UPDATE_INTERVAL 10000 // Number of milliseconds between relay updates
#define RELAY_INIT_DELAY 30000      // Number of milliseconds from start before relay can be switched on (must be less than RELAY_CYCLE_INTERVAL)
#define RELAY_CYCLE_INTERVAL 300000 // Number of milliseconds between relay-on cycles
#define TEMP_UPDATE_INTERVAL 1000   // Number of milliseconds between temperature sensor readings
#define TEMP_AVERAGE_WINDOW 60000   // Number of milliseconds over which to average temperature
#define TEMP_HISTORY_COUNT (TEMP_AVERAGE_WINDOW/TEMP_UPDATE_INTERVAL) // Number of measurements to keep
#define TEMP_FAIL_MAX 4 // Number of times sensor read must fail consecutively to trigger relay-off

#define THERMO_CLK_PIN  14
#define THERMO_CS_PIN   15
#define THERMO_DO_PIN   12

#define RELAY_PIN 13
#define RELAY_ON HIGH
#define RELAY_OFF LOW

#define ONBOARD_LED 2

float temp_setpoint = DEFAULT_TEMP_SETPOINT;
float temp_differential = DEFAULT_TEMP_DIFFERENTIAL;
float temp_correction = DEFAULT_TEMP_CORRECTION;
float temp_current = 0.0f;
String temp_unit = "F";

// Interval in ms of the reads
long interval = 5000;
long last_update = 0;
long last_publish = 0;
unsigned long last_relay_update_time = 0;
unsigned long last_relay_on_time = 0;

float temp_history[TEMP_HISTORY_COUNT];
int temp_history_index = 0;
int temp_history_count = 0;
int temp_fail_count = 0;
bool relay_state = false;
bool wifi_connected = false;

// Prefix for the MQTT Client Identification
String client_id = "esp32-client-";

// Init WiFi/WiFiUDP, NTP and MQTT Client
WiFiUDP ntpUDP;
WiFiClient espClient;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 3600000);
void mqtt_callback(char *topic, byte *message, unsigned int length);
PubSubClient mqttClient(mqtt_server, mqtt_port, mqtt_callback, espClient);

// Persistent storage
Preferences preferences;

//MAX6675 sensor(SELECT_PIN, DATA_PIN, CLOCK_PIN);
MAX6675 sensor(THERMO_CLK_PIN, THERMO_CS_PIN, THERMO_DO_PIN);


/**
 * Reconnect to MQTT Broker
 */
void mqtt_reconnect() {

    Log.notice(F("Attempting MQTT connection to %s" CR), mqtt_server);

    // Turn on led board
    digitalWrite(ONBOARD_LED, HIGH);

    // Attempt to connect
    if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Log.notice(F("Connected as client_id %s :-)" CR), client_id.c_str());

        // Subscribe to setpoint control topic
        {
            String topic = String(mqtt_root) + "/" + device_name + "/control/setpoint";
            mqttClient.subscribe(topic.c_str(), 1);
            Log.notice(F("Subscribed to control topic %s " CR), topic.c_str());
        }

        // Subscribe to settings control topic
        {
            String topic = String(mqtt_root) + "/" + device_name + "/control/settings";
            mqttClient.subscribe(topic.c_str(), 1);
            Log.notice(F("Subscribed to control topic %s " CR), topic.c_str());
        }
    }
    else {
        Log.error(F("MQTT connection failed, state=%d" CR), mqttClient.state());
        // Turn off led board and wait 5 seconds before retrying
        digitalWrite(ONBOARD_LED, LOW);
    }
}

void wifi_event(WiFiEvent_t event) {

    switch (event) {

        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.println("WiFi connected!");
            Serial.print("IP Address: ");
            Serial.print(WiFi.localIP());
            Serial.println("");
            Serial.print("Mac Address: ");
            Serial.print(WiFi.macAddress());
            Serial.println("");
            Serial.print("Hostname: ");
            Serial.print(WiFi.getHostname());
            Serial.println("");
            Serial.print("Gateway: ");
            Serial.print(WiFi.gatewayIP());
            Serial.println("");

            wifi_connected = true;

            // When setup wifi ok turn on led board
            //digitalWrite(ONBOARD_LED, HIGH);

            // Initialize NTP
            timeClient.begin();
            timeClient.setTimeOffset(0);
            Serial.print("Pre NTP Time: ");
            Serial.println(timeClient.getFormattedTime());
            // Wait for NTP time sync
            //while (!timeClient.forceUpdate()) {
            //  delay(1000);
            //  Serial.println("Retrying NTP update...");
            //}
            // Force NTP time sync
            timeClient.forceUpdate();
            Serial.print("NTP Time: ");
            Serial.println(timeClient.getFormattedTime());

            // Connect to MQTT broker
            mqtt_reconnect();

            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            Log.notice(F("WiFi lost. Attempting reconnect to WiFi network: %s" CR), ssid);
            wifi_connected = false;
            WiFi.begin(ssid, password);  // Reconnect
            break;

        default:
            break;
    }
}

/**
 * Setup the WiFi Connection
 */
void setup_wifi() {

    // We start by connecting to a WiFi network
    Log.notice(F("Connecting to WiFi network: %s" CR), ssid);

    WiFi.onEvent(wifi_event);  // Attach event handler
    WiFi.begin(ssid, password);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(client_id.c_str());
}

void read_settings() {
    interval = preferences.getLong("interval", DEFAULT_INTERVAL);
    temp_setpoint = preferences.getFloat("setpoint", DEFAULT_TEMP_SETPOINT);
    temp_differential = preferences.getFloat("differential", DEFAULT_TEMP_DIFFERENTIAL);
    temp_correction = preferences.getFloat("correction", DEFAULT_TEMP_CORRECTION);
    temp_unit = preferences.getString("unit", "F");
}

void write_settings() {
    preferences.putLong("interval", interval);
    preferences.putFloat("setpoint", temp_setpoint);
    preferences.putFloat("differential", temp_differential);
    preferences.putFloat("correction", temp_correction);
    preferences.putString("unit", temp_unit);
}

/**
 * Setup lifecycle
 */
void setup()
{

    // Setup PIN mode for relay
    //pinMode(RELAY_PIN, INPUT_PULLUP);    // Set high immediately to avoid momentary relay activation
    //digitalWrite(RELAY_PIN, HIGH);
    //pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);
    pinMode(RELAY_PIN, OUTPUT);

    // Setup PIN Mode for onboard LED
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW);

    Serial.begin(115200);
    Serial.println("Starting up...");

    // Initialize with log level and log output.
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);

    // Initialize last relay-on time to effect init delay
    last_relay_on_time = millis() - (RELAY_CYCLE_INTERVAL - RELAY_INIT_DELAY);

    // Initialize persistence
    preferences.begin("settings", false);
    //preferences.clear();
    read_settings();

    // Log ESP Chip information
    Log.notice(F("ESP32 Chip model %s Rev %d" CR), ESP.getChipModel(), ESP.getChipRevision());
    Log.notice(F("This chip has %d cores" CR), ESP.getChipCores());

    client_id += String(random(0xffff), HEX);

    // Connect to WiFi
    setup_wifi();
}


// Set relay state to on
void set_relay_on() {
  if (relay_state) {
    Serial.println("--- ABOVE THRESHOLD, LEAVING RELAY ON ---");
    return;
  }
  unsigned long current_time = millis();
  //Serial.print("relay-on interval: ");
  //Serial.println(current_time - last_relay_on_time);
  if ((current_time - last_relay_on_time) < RELAY_CYCLE_INTERVAL) {
    Serial.print("--- ABOVE THRESHOLD, WAITING ");
    Serial.print((RELAY_CYCLE_INTERVAL - (current_time - last_relay_on_time)) / 1000);
    Serial.println("s (CYCLE DELAY) TO TURN RELAY ON ---");
  }
  else {
    Serial.println("--- ABOVE THRESHOLD, TURNING RELAY ON ---");
    digitalWrite(RELAY_PIN, RELAY_ON); // relay on
    //digitalWrite(LED_BUILTIN, HIGH); // led on
    last_relay_on_time = current_time;
    relay_state = true;
  }
}

// Set relay state to off
void set_relay_off() {
  if (!relay_state) {
    Serial.println("--- BELOW THRESHOLD, LEAVING RELAY OFF ---");
    return;
  }
  Serial.println("--- BELOW THRESHOLD, TURNING RELAY OFF ---");
  digitalWrite(RELAY_PIN, RELAY_OFF); // relay off
  //digitalWrite(LED_BUILTIN, HIGH); // led off
  relay_state = false;
}

float normalize_temp(float temp, String unit, String target_unit) {
    String from_unit(unit);
    String to_unit(target_unit);
    from_unit.toUpperCase();
    to_unit.toUpperCase();
    if (from_unit == to_unit) {
        return temp;
    }
    if (from_unit == "F" && to_unit == "C") {
        return (temp - 32.0f) * (5.0f/9.0f);
    }
    else if (from_unit == "C" && to_unit == "F") {
        return temp * (9.0f/5.0f) + 32.0f;
    }
    return 0.0f;
}

float normalize_diff_temp(float diff_temp, String unit, String target_unit) {
    String from_unit(unit);
    String to_unit(target_unit);
    from_unit.toUpperCase();
    to_unit.toUpperCase();
    if (from_unit == to_unit) {
        return diff_temp;
    }
    if (from_unit == "F" && to_unit == "C") {
        return diff_temp * (5.0f/9.0f);
    }
    else if (from_unit == "C" && to_unit == "F") {
        return diff_temp * (9.0f/5.0f);
    }
    return 0.0f;
}

// Update the temperature history and return the running average
void update_temp(float temp) {
    temp_history[temp_history_index] = temp;
    temp_history_index = (temp_history_index + 1) % TEMP_HISTORY_COUNT;
    if (temp_history_count < TEMP_HISTORY_COUNT) {
        temp_history_count++;
    }
}

// Calculate average of the stored temperatures
float get_temp_avg() {
    float sum = 0.0;
    for (int i = 0; i < temp_history_count; i++) {
        sum += temp_history[i];
    }

    return sum / temp_history_count;
}

// Calculate standard deviation of the stored temperatures
float get_temp_stddev() {
    if (temp_history_count == 0) return 0.0;

    float mean = 0.0;
    for (int i = 0; i < temp_history_count; i++) {
        mean += temp_history[i];
    }
    mean /= temp_history_count;

    float variance = 0.0;
    for (int i = 0; i < temp_history_count; i++) {
        float diff = temp_history[i] - mean;
        variance += diff * diff;
    }
    variance /= temp_history_count;  // For population stddev. Use (count - 1) for sample stddev

    return sqrt(variance);
}

void publishTemperature() {

    String topic = String(mqtt_root) + "/" + device_name + "/state/temperature";

    JsonDocument doc;
    doc["current"] = normalize_temp(temp_current, "C", temp_unit);
    doc["average"] = normalize_temp(get_temp_avg(), "C", temp_unit);
    doc["stddev"] = normalize_diff_temp(get_temp_stddev(), "C", temp_unit);
    doc["unit"] = temp_unit;
    doc["timestamp"] = timeClient.getEpochTime();

    String data;
    serializeJson(doc, data);

    mqttClient.publish(topic.c_str(), data.c_str());

    //Serial.println(topic.c_str());
    //serializeJsonPretty(doc, Serial);
    //Serial.println();
}

void publishRelay() {

    String topic = String(mqtt_root) + "/" + device_name + "/state/relay";

    JsonDocument doc;
    doc["value"] = digitalRead(RELAY_PIN) == RELAY_ON ? 1 : 0;
    doc["timestamp"] = timeClient.getEpochTime();

    String data;
    serializeJson(doc, data);

    mqttClient.publish(topic.c_str(), data.c_str());

    //Serial.println(topic.c_str());
    //serializeJsonPretty(doc, Serial);
    //Serial.println();
}

void publishSetpoint() {

    String topic = String(mqtt_root) + "/" + device_name + "/state/setpoint";

    JsonDocument doc;
    doc["value"] = normalize_temp(temp_setpoint, "C", temp_unit);
    doc["unit"] = temp_unit;
    doc["timestamp"] = timeClient.getEpochTime();

    String data;
    serializeJson(doc, data);

    mqttClient.publish(topic.c_str(), data.c_str());

    //Serial.println(topic.c_str());
    //serializeJsonPretty(doc, Serial);
    //Serial.println();
}

void publishSettings() {

    String topic = String(mqtt_root) + "/" + device_name + "/state/settings";

    JsonDocument doc;
    doc["unit"] = temp_unit;
    doc["differential"] = normalize_diff_temp(temp_differential, "C", temp_unit);
    doc["correction"] = normalize_diff_temp(temp_correction, "C", temp_unit);
    doc["interval"] = interval;
    doc["timestamp"] = timeClient.getEpochTime();

    String data;
    serializeJson(doc, data);

    mqttClient.publish(topic.c_str(), data.c_str());

    //Serial.println(topic.c_str());
    //serializeJsonPretty(doc, Serial);
    //Serial.println();
}

/**
 * Loop lifecycle
 */
void loop() {

    if (wifi_connected) {
        // Run NTP update loop
        timeClient.update();

        // Run MQTT client loop
        mqttClient.loop();
    }

    long now = millis();

    // Perform teperature/relay management
    if ((now - last_update) > TEMP_UPDATE_INTERVAL) {
        last_update = now;

        // Read temperature from sensor
        //int status = sensor.read();
        //temp_current = sensor.readFahrenheit() + temp_correction;
        temp_current = sensor.readCelsius() + temp_correction;

        //if (status == 0) {
        if (true) {
            // Update temperature history
            temp_fail_count = 0;
            update_temp(temp_current);
            float temp_avg = get_temp_avg();
            Serial.print("temp current: ");
            Serial.print(normalize_temp(temp_current, "C", temp_unit));
            Serial.print(" ");
            Serial.print(temp_unit.c_str());
            Serial.print(", stddev: ");
            Serial.print(normalize_diff_temp(get_temp_stddev(), "C", temp_unit));
            Serial.print(" ");
            Serial.print(temp_unit.c_str());
            Serial.print(", avg: ");
            Serial.print(normalize_temp(temp_avg, "C", temp_unit));
            Serial.print(" ");
            Serial.print(temp_unit.c_str());
            Serial.print(", setpoint: ");
            Serial.print(normalize_temp(temp_setpoint, "C", temp_unit));
            Serial.print(" ");
            Serial.print(temp_unit.c_str());
            Serial.print(", relay: ");
            Serial.println(relay_state ? "on" : "off");

            // Display cuurrent average temperature
            //matrix.print(temp_avg);
            //matrix.writeDisplay();

            // Set state of thermostat relay
            unsigned long current_time = millis();
            if ((current_time - last_relay_update_time) >= RELAY_UPDATE_INTERVAL) {
                if (temp_avg > (temp_setpoint + temp_differential)) {
                set_relay_on();
                }
                else if (temp_avg < temp_setpoint) {
                set_relay_off();
                }
                else {
                Serial.println("--- WITHIN THRESHOLD, LEAVING RELAY AS-IS ---");
                }
                last_relay_update_time = current_time;
            }
        }
        else {
            // Error occurred reading temperature sensor
            ++temp_fail_count;
            Serial.println("Error reading temperature!");
            //matrix.print(10000, DEC);
            //matrix.writeDisplay();
            // Turn relay off if max temperature read failures is reached
            if (temp_fail_count > TEMP_FAIL_MAX) {
                set_relay_off();
            }
        }
    }

    // Publish messages
    if ((now - last_publish) > interval) {
        last_publish = now;

        // Check MQTT client connection
        if (wifi_connected && !mqttClient.connected()) {
            mqtt_reconnect();
        }

        // Publish if connected
        if (wifi_connected && mqttClient.connected()) {

            // Turn onboard led off before sending
            digitalWrite(ONBOARD_LED, LOW);

            publishTemperature();
            publishSetpoint();
            publishRelay();

            // Turn onboard led on after sending
            delay(100);
            digitalWrite(ONBOARD_LED, HIGH);
        }
    }

}


/**
  * MQTT Callback
  * 
  * If a message is received on the topic esp32/command (es. Relay off or on).
  * Format: {$device-name}:{relay;$relayId;$command}
  * Es: 
  *  esp32-zone-1:relay;3;off (switch off relay 3 of the specified device)
  *  esp32-zone-1:relay;2;on (switch on relay 2 of the specified device)
  *  esp32-zone-1:relay;3;status (get status of the relay 3 of the specified device) 
  */
void mqtt_callback(char *topic, byte *message, unsigned int length)
{

    Log.info(F("Message arrived on topic: %s length: %d" CR), topic, length);

    String topicStr(topic);
    String topicPrefix = String(mqtt_root) + "/" + device_name + "/control/";
    if (topicStr.indexOf(topicPrefix) != 0) {
        Log.warning(F("Unrecognized topic: %s" CR), topicStr.c_str());
        return;
    }
    String name = topicStr.substring(topicPrefix.length());

    String messageStr((const char*)message, length);
    Log.info(F("Message Content: \"%s\"" CR), messageStr.c_str());

    if (name == "setpoint") {
        if (length == 0 || message == nullptr) {
            Log.warning(F("Missing message" CR));
            return;
        }
        JsonDocument doc;
        deserializeJson(doc, message);

        Serial.println(topicStr.c_str());
        serializeJsonPretty(doc, Serial);
        Serial.println();

        String unit = temp_unit;
        if (doc["unit"].is<String>()) {
            unit = doc["unit"].as<String>();
        }
        if (doc["value"].is<float>()) {
            temp_setpoint = normalize_temp(doc["value"], unit, "C");
            Serial.println(temp_setpoint);
            Log.info(F("Updated setpoint to %F %s" CR), temp_setpoint, unit.c_str());
            write_settings();
        }
    }
    else if (name == "settings") {
        if (length == 0 || message == nullptr) {
            publishSettings();
            return;
        }
        JsonDocument doc;
        deserializeJson(doc, message);

        Serial.println(topicStr.c_str());
        serializeJsonPretty(doc, Serial);
        Serial.println();

        if (doc["unit"].is<String>()) {
            temp_unit = doc["unit"].as<String>();
            Log.info(F("Updated temperature unit to %s" CR), temp_unit.c_str());
        }
        if (doc["differential"].is<float>()) {
            temp_differential = normalize_diff_temp(doc["differential"], temp_unit, "C");
            Log.info(F("Updated temperature differential to %F %s" CR), temp_differential, temp_unit.c_str());
        }
        if (doc["correction"].is<float>()) {
            temp_correction = normalize_diff_temp(doc["correction"], temp_unit, "C");
            Log.info(F("Updated temperature correction to %F %s" CR), temp_correction, temp_unit.c_str());
        }
        if (doc["interval"].is<long>()) {
            interval = doc["interval"];
            Log.info(F("Updated interval to %d" CR), interval);
        }

        write_settings();

        publishSettings();
    }
}
