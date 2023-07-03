//This code detects the nearest BLE of an ESP32-LilyGO-T-ETH-POE.

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <math.h>
#include <map>
#include <Wire.h>

#define MEASURED_POWER -79  // Potencia de señal medida a 1 metro (en dBm)
#define N 2.5               // Factor de atenuación
#define SCAN_INTERVAL_MS 5000 // Intervalo de tiempo para realizar otro escaneo (en milisegundos)

BLEScan* pBLEScan;
BLEScanResults foundDevices;

static unsigned long lastScanTime = 0;
unsigned long currentTime = 0;
float closestDistance = 0;
int counter = 0;
int mostCommonCount = 0;
float distance = 0;
int count = 0;
String deviceName;
const char* deviceID;
const char* closestBLE;

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Buscando Dispositivos...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

}

void loop() {
    BLEcercano();
}

void BLEcercano(){
    lastScanTime = 0;
    currentTime = millis();
  
    if (currentTime - lastScanTime >= SCAN_INTERVAL_MS) {
      lastScanTime = currentTime;
  
      foundDevices = pBLEScan->start(SCAN_INTERVAL_MS /1000);
      count = foundDevices.getCount();
      Serial.println("");
      Serial.print("Se encontró ");
      Serial.print(count);
      Serial.println(" dispositivo(s).");
      BLEAdvertisedDevice closestDevice;
      closestDistance = 100.0; // Inicialmente, se establece una distancia muy grande
      for (int i = 0; i < count; i++) {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);
        deviceName = device.getName().c_str();
        deviceID = device.getAddress().toString().c_str();
        if (deviceName.substring(0,2)=="KM") { // Filtrar por nombres que comiencen con "KM" o IDs específicos
          distance = calculateDistance(device.getRSSI());
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print("ID: ");
          Serial.print(deviceID);
          Serial.print(", Nombre: ");
          Serial.print(deviceName );
          Serial.print(", RSSI: ");
          Serial.print(device.getRSSI());
          Serial.print(" Distancia: ");
          Serial.print(distance);
          Serial.println(" metros");
          if (distance < closestDistance) {
            closestDevice = device;
            closestDistance = distance;
          }
        }
      }     
       if (count > 0) {
        
        Serial.print("Dispositivo más cercano: ");
        Serial.print("ID: ");
        Serial.print(closestDevice.getAddress().toString().c_str());
        Serial.print(", Nombre: ");
        Serial.print(closestDevice.getName().c_str());
        Serial.print(", RSSI: ");
        Serial.print(closestDevice.getRSSI());
        Serial.print(" Distancia: ");
        Serial.print(closestDistance);
        Serial.println(" metros");        
      
      }
        static std::map<std::string, int> deviceNames; // Mapa para almacenar los nombres del dispositivo más cercano y su conteo
        if (counter >= 5) {
          
          // Calcular el nombre del dispositivo más común en los últimos 10 escaneos
          std::string mostCommonName;
          mostCommonCount = 0;
          for (auto& entry : deviceNames) {
            if (entry.second > mostCommonCount) {
              mostCommonName = entry.first;
              mostCommonCount = entry.second;
            }
          }

          closestBLE = mostCommonName.c_str();
          Serial.print("Llanta promedio: ");
          Serial.println(closestBLE);
          deviceNames.clear(); // Borrar el mapa para comenzar a contar los nombres en los próximos 10 escaneos  
          counter=0;        
        }
    counter++;
    //Agregar el nombre del dispositivo más cercano al mapa
    std::string closestDeviceName = closestDevice.getName().c_str();
    deviceNames[closestDeviceName]++;
    pBLEScan->clearResults();  
    
    }
}

float calculateDistance(int rssi) {
  return pow(10, ((MEASURED_POWER - rssi) / (10 * N)));
}
