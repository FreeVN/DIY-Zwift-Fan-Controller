# DIY-Zwift-Fan-Controller
<b>(ESP32 Ble-MQTT Gateway)</b>
	
Created with the intention to control a 3-speed fan in function of heartrate/power/cadence for a comfortable Zwift experience.
	
## Hardware
		ESP32 board (Wemos Lolin 32 OLED)
		Solid state relais module
		RPI 4 (as a Docker host for Node Red, InfluxBD, Grafana, MQTT server, ...)
## What is does
Grabs Data from indoor bike trainer and heartrate monitor BLE-devices (Elite Direto & Wahoo Tickr in this case, but doesn't matter) 
Sends it to MQTT -> Node Red -> InfluxDB.

Node Red handles the fanspeed depending on heartrate & environmental conditions (outside temp/humidity) and sends it to the ESP32 (MQTT) wich then operates the solid state relais.

Grafana pulls data from InfluxDB to create a (rather beautiful) dashboard.

## Important note
Botched together with little expierence, but a lot of inspiration from https://github.com/Merdeka, https://github.com/chegewara, https://github.com/SensorsIot, https://github.com/espressif/arduino-esp32.
