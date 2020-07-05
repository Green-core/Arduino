#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "Dialog 4G";
const char* password = "0GN80THE415";

void setup(){
  Serial.begin(9600);
  WiFi.begin(ssid, password);

  while(WiFi.status()!=WL_CONNECTED){
    delay(1000);
    Serial.println("Connecting..");

  }
}

void loop(){
//  if(WiFi.status()==WL_CONNECTED){ //Check WiFi connection status
//    Serial.println("Connected");
//
//    HTTPClient http;  //Declare an object of class HTTPClient
//
//    String postData = "id=5ec66db7aa16ff3a80870c9a";  // module id
//
//    http.begin("http://192.168.8.108:3002/units/get-actuator-status");  //get actuator status - wether the actuators should be activated or not
//    http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
//    int httpCode = http.POST(postData);       //Send the request
//
//    if(httpCode>0){ //Check the returning code
//      
//      DynamicJsonDocument doc(1024);
//      DeserializationError error = deserializeJson(doc, http.getString());  // get the request response payload
//      
//      if(error){
//         Serial.print("deserializeJson failed with code");
//         Serial.println(error.c_str()); // print the error
//      }
//      else{
//        bool waterMotorActuator = doc["waterMotorActuator"];
//        bool lightActuator = doc["lightActuator"];
//        bool buzzerActuator = doc["buzzerActuator"];
//        bool fertilizerActuator = doc["fertilizerActuator"];
//        // Output to serial monitor
//        Serial.print("waterMotorActuator:");
//        Serial.println(waterMotorActuator);
//        Serial.print("lightActuator:");
//        Serial.println(lightActuator);
//        Serial.print("buzzerActuator:");
//        Serial.println(buzzerActuator);
//        Serial.println("fertilizerActuator:");
//        Serial.println(fertilizerActuator);
//
//        if (lightActuator /* && lightIntensity // less than certain value*/) {
//          Serial.println("Switchin on Lights");
//          // switch on lights
//        }
//        else {
//          // switch off lights
//        }
//
//
//        // send data
//        // soilMoisture
//        // humidity
//        // temperature
//        // lightIntensity
//      }
//    }
//    
//    http.end();   //Close connection
//
//
//    
//
//  }
//  else {
//
//  }
//
  delay(30000);    //Send a request every 30 seconds

  
  if(WiFi.status()==WL_CONNECTED){ //Check WiFi connection status
    Serial.println("Connected");

    HTTPClient http;  //Declare an object of class HTTPClient

    String moduleID = "5ec66db7aa16ff3a80870c9a";
    String soilMoisture = String(random(90, 100)) + ".00";
    String humidity = String(random(10, 90)) + "%";
    String temperature = String(random(10, 50)) + "C";
    String lightIntensity = String(random(0, 1000)) + "lux";

    String postData = "moduleID=" + moduleID + "&soilMoisture=" + soilMoisture +  "&humidity=" + humidity + "&temperature=" + temperature + "&lightIntensity=" + lightIntensity;
//    String postData = "moduleID=5ec66db7aa16ff3a80870c9a&soilMoisture=70.00&humidity=30%&temperature=37C&lightIntensity=200lux";
    Serial.println(postData);
    http.begin("http://192.168.8.108:3002/units/update-data");  //get actuator status - wether the actuators should be activated or not
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
    int httpCode = http.POST(postData);       //Send the request

    if(httpCode>0){ //Check the returning code
      
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, http.getString());  // get the request response payload
      
      if(error){
         Serial.print("deserializeJson failed with code");
         Serial.println(error.c_str()); // print the error
      }
      else{
        Serial.println("Sensor data updated");
      }
    }
    
    http.end();   //Close connection


    

  }
  else {

  }

  delay(30000);    //Send a request every 30 seconds
}
