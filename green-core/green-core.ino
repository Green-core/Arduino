//#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <dht.h>
//WiFiClient client;

#define dht_apin A6 // Analog Pin sensor is connected to
dht DHT;

#define lightSensorPin A7
#define growLightPin 6
//const char* ssid     = "Dialog 4G";         // The SSID (name) of the Wi-Fi network you want to connect to
//const char* pass = "0GN80THE415";     // The password of the Wi-Fi network

void setup() {
  Serial.begin(9600);
//  delay(10);

//  Serial.println("Connecting to ");
//  Serial.println(ssid);

//  WiFi.begin(ssid, pass);
//  while (WiFi.status() != WL_CONNECTED)
//  {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.println("WiFi connected");
  
  delay(500);//Delay to let system boot
  Serial.println("DHT11 Humidity & temperature Sensor\n\n");
  delay(1000);//Wait before accessing Sensor
}

void loop() {

  // Humidity & Temperature
  DHT.read11(dht_apin);
  Serial.print("Current humidity = ");
  Serial.print(DHT.humidity);
  Serial.print("%  ");
  Serial.print("temperature = ");
  Serial.print(DHT.temperature);
  Serial.println("C  ");
  delay(5000);
//
//  // Light Intensity
  int lightSensorReading;
  lightSensorReading = analogRead(lightSensorPin);
  Serial.println("Light Intensity : ");
  Serial.println(lightSensorReading);
  if (lightSensorReading > 500) // change the lux value. and change the sensor to a analog one.
    digitalWrite(growLightPin, HIGH);
  else
    digitalWrite(growLightPin, LOW);
}
