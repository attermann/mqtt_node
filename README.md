# mqtt_sensor

ESP32 MQTT Sensor Node

## Config

Configure WiFi/MQTT/node details in `.env` file before building.

## Example Mosquitto CLI

Monitor all nodes:

```
mosquitto_sub -v -h raspberrypi.local -t 'node/#'
```

Monitor single node:

```
mosquitto_sub -v -h raspberrypi.local -t 'node/kegerator/#'
```

Update node settings:

```
mosquitto_pub -h raspberrypi.local -t 'node/kegerator/control/settings' -m '{"unit": "F", "differential":" 5.0, "interval": 10000}'
```

Update node thermostat setpoint:

```
mosquitto_pub -h raspberrypi.local -t 'node/kegerator/control/setpoint' -m '{"value": 37.0, "unit": "F"}''
```
