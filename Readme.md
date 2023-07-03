## BLUETOOTH LOW ENERGY

This is a repository to detect the nearest BLE based in power signal strength (dB) each t seconds. Also, sends the most common nearest BLE in n samples trough WiFi and Ethernet to a MQTT database.    

First, install ESP32 library adding the next line into Preferences. <br />
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Then, go to Manage Libraries section and install the ESP32 library.

PubSubClient and WiFi libaries are also needed. 
