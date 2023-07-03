// Codigo que publica en "datos/bno055" el valor del angulo.

#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <PubSubClient.h>
//#define WIFI_SSID  "ACME0"//  Cambiar por el nombre de tu red Wi-Fi
//#define WIFI_PASSWORD  "Electronic2016!"//  Cambiar por la contraseña de tu red Wi-Fi
#define MQTT_SERVER  "192.168.82.10"//  Cambiar por la dirección IP de la Raspberry Pi
#define MQTT_PORT 1883
#define MQTT_TOPIC "datos/frontal_derecho"

//Adafruit_BNO055 bno = Adafruit_BNO055();

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

  char message[100];
  float a = 1.23 ;
  
void setup() {
  Serial.begin(115200);
  
  // Conexión a Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a Wi-Fi...");
  }
  Serial.println("Conectado a Wi-Fi");

  // Conexión a MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32-BNO055-1")) {
      Serial.println("Conectado a MQTT");
    } else {
      Serial.println("Error al conectar a MQTT");
      delay(1000);
    }
  }

  // Inicialización del sensor BNO055
//  if (!bno.begin()) {
//    Serial.println("Error al inicializar el sensor BNO055");
//    while (1);
//  }
}

void loop() {
  // Lectura de los datos del sensor BNO055
  //sensors_event_t event;
  //bno.getEvent(&event);
  
  // Envío de los datos a MQTT
  a++;
  if (mqttClient.connected()) {
    // Limitamos solamente al eje que deseamos.
    sprintf(message, "%.2f", a);
    mqttClient.publish(MQTT_TOPIC, message);
    Serial.println(a);
  }
  else{
      while (!mqttClient.connected()) {
        if (mqttClient.connect("ESP32-BNO055-1")) {
          Serial.println("Conectado a MQTT");
        } 
        else {
          Serial.println("Error al conectar a MQTT");
          delay(1000);
         }
      }
  }
  
  //Serial.println(event.orientation.y);
  // Espera de 10 ms antes de volver a leer los datos del sensor
  delay(1000);
}
