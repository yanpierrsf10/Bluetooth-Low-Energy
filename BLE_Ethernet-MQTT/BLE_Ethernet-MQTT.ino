//This code detects the nearest BLE of an ESP32-LilyGO-T-ETH-POE and sends it to a database through MQTT using an Ethernet connection.

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <ETH.h>
#include <SPI.h>
#include <SD.h>
#include "time.h"
#include "sntp.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <math.h>
#include <map>

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN -1
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR 0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN 23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN 18
#define NRST        5
#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define ESP_MQTT_TAG "ESP32-IZQUIERDO"
#define MEASURED_POWER -79  // Potencia de señal medida a 1 metro (en dBm)
#define N 2.5               // Factor de atenuación
#define SCAN_INTERVAL_MS 7000 // Intervalo de tiempo para realizar otro escaneo (en milisegundos)
#define WIFI_SSID  "ACME0"//  Cambiar por el nombre de tu red Wi-Fi
#define WIFI_PASSWORD  "Electronic2016!"//  Cambiar por la contraseña de tu red Wi-Fi
#define MQTT_SERVER  "192.168.82.10"//  Cambiar por la dirección IP de la Raspberry Pi
#define MQTT_PORT 1883
#define MQTT_TOPIC "datos/frontal_izquierdo"

char message[100];
static bool eth_connected = false;
static unsigned long lastScanTime = 0;
unsigned long currentTime = 0;
int closestRSSI = 0;
int counter = 0;
int mostCommonCount = 0;
int deviceRSSI = 0;
int count = 0;
String deviceName;
const char* deviceID;
const char* closestBLE;
String mostCommonName;
String closestDevice;
BLEScan* pBLEScan;
BLEScanResults foundDevices;

WiFiClient ethClient;
PubSubClient mqttClient(ethClient);

void WiFiEvent(WiFiEvent_t event)
{
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
        Serial.println("ETH Started");
        //set eth hostname here
        ETH.setHostname("esp32-ethernet");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        eth_connected = true;
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("ETH MAC: ");
        Serial.print(ETH.macAddress());
        Serial.print(", IPv4: ");
        Serial.println(ETH.localIP());
        eth_connected = true;
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        eth_connected = false;
        break;
    case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        eth_connected = false;
        break;
    default:
        break;
    }
}

void publisher(const char* input)
{
    Serial.println("Enviando a MQTT...");
    sprintf(message,input);
    mqttClient.publish(MQTT_TOPIC, message); 
    Serial.print("Llanta promedio: ");
    Serial.println(input);
}

void reconnect(){
  // Conexion Ethernet
  pinMode(SD_MISO, INPUT_PULLUP);
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  WiFi.onEvent(WiFiEvent);
  pinMode(NRST, OUTPUT);
  digitalWrite(NRST, 0);
  delay(200);
  digitalWrite(NRST, 1);
  delay(200);
  digitalWrite(NRST, 0);
  delay(200);
  digitalWrite(NRST, 1); 
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN,
            ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
               
  sntp_servermode_dhcp(1);
  
  WiFi.mode(WIFI_STA);// Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect();
  delay(100);
  // Conexión a MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.connect(ESP_MQTT_TAG);

}

void BLE_Closest(){
    lastScanTime = 0;
    currentTime = millis();
  
    if (currentTime - lastScanTime >= SCAN_INTERVAL_MS) {
      lastScanTime = currentTime;
  
      foundDevices = pBLEScan->start(SCAN_INTERVAL_MS /1000);
      count = foundDevices.getCount();
      
      closestRSSI = -100; // Inicialmente, se establece una distancia muy grande
  
      for (int i = 0; i < count; i++) {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);
        deviceName = device.getName().c_str();
        
        if (deviceName.substring(0,2)=="KM") { // Filtrar por nombres que comiencen con "KM" o IDs específicos
            deviceRSSI = device.getRSSI();          
          if (deviceRSSI > closestRSSI) {
            closestDevice = device.getName().c_str();
            closestRSSI = deviceRSSI;
          }
        }
      }
        
    counter++;
    if (count > 0) {
        Serial.print(counter);
        Serial.print("BLE: ");
        Serial.println(closestDevice);         
    }
    static std::map<std::string, int> deviceNames; // Mapa para almacenar los nombres del dispositivo más cercano y su conteo
    if (counter >= 5) {      
          // Calcular el nombre del dispositivo más común en los últimos N escaneos
          std::string mostCommonName;
          mostCommonCount = 0;
          for (auto& entry : deviceNames) {
            if (entry.second > mostCommonCount) {
              mostCommonName = entry.first;
              mostCommonCount = entry.second;
            }
          }
          
          closestBLE = mostCommonName.c_str();
          mqttClient.connect(ESP_MQTT_TAG);
          if(mqttClient.connected()){
            publisher(closestBLE);
          }
          else{
            reconnect();
          }
          deviceNames.clear(); // Borrar el mapa para comenzar a contar los nombres en los próximos 10 escaneos  
          counter=0;        
    }
    
    //Agregar el nombre del dispositivo más cercano al mapa
    std::string closestDeviceName = closestDevice.c_str();
    deviceNames[closestDeviceName]++;
    pBLEScan->clearResults();  
    
    }
}


void setup() {
  Serial.begin(115200);
  // Conexion Ethernet
  pinMode(SD_MISO, INPUT_PULLUP);
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  WiFi.onEvent(WiFiEvent);
    
  pinMode(NRST, OUTPUT);
  digitalWrite(NRST, 0);
  delay(200);
  digitalWrite(NRST, 1);
  delay(200);
  digitalWrite(NRST, 0);
  delay(200);
  digitalWrite(NRST, 1);
    
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN,
            ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

  while (!eth_connected) {
    Serial.println("Wait eth connect..."); delay(2000);
  }
   
  sntp_servermode_dhcp(1);

    
  WiFi.mode(WIFI_STA);// Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect();
  delay(100);
    
  // Conexión a MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.connect(ESP_MQTT_TAG);
  while (!mqttClient.connected()) {
    if (mqttClient.connected()) {
        Serial.println("Conectado a MQTT");
    } 
    else {
        Serial.println("Error al conectar a MQTT");
        delay(1000);
        mqttClient.connect(ESP_MQTT_TAG);
    }
  }

  //Scaneo 
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
}
void loop() {
  BLE_Closest();
}
