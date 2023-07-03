## BLUETOOTH LOW ENERGY

This is a repository to detect the nearest BLE based on power signal strength (dB) every t seconds. Also sends the most common nearest BLE in n samples through WiFi and Ethernet to a MQTT database.    

First, install the ESP32 library, adding the next line into Preferences. <br />
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Then, go to the Manage Libraries section and install the ESP32 library.

PubSubClient and WiFi libraries are also needed.

**Board**: AI Thinker ESP32-CAM