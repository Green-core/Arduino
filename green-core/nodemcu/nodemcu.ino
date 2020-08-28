#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#define ARDUINOJSON_ENABLE_COMMENTS 1
#include <ArduinoJson.h>
#include <TimedAction.h>
#include <SoftwareSerial.h>
SoftwareSerial s(12,14);  // 6-Rx , 5-Tx

const char* ssid = "Dialog 4G";
const char* password = "0GN80THE415";

String moduleId = "5ec66db7aa16ff3a80870c9a";

bool waterMotorActuator = false;
bool lightActuator = false;
bool buzzerActuator = false;
bool fertilizerActuator = false;
bool automated = true;

String soilMoistureReading;
String humidityReading;
String temperatureReading;
String lightIntensityReading;

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  
  while (!Serial) continue;

  WiFi.begin(ssid, password);

  while(WiFi.status()!=WL_CONNECTED){
    delay(1000);
    Serial.println("Connecting..");

  }
}

void recieveActuatorStatus(){
  if(WiFi.status()==WL_CONNECTED){ //Check WiFi connection status
    Serial.println("Connected");

    HTTPClient http;  //Declare an object of class HTTPClient

    String postData = "id=" + moduleId;  // module id
    
//    http://192.168.8.104:3002
//    https://gciobe.herokuapp.com

    http.begin("http://192.168.8.104:3002/units/get-actuator-status");  //get actuator status - wether the actuators should be activated or not
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header

    http.useHTTP10(true);
    int httpCode = http.POST(postData);       //Send the request

    if(httpCode>0){ //Check the returning code
      
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, http.getString());  // get the request response payload
      
      if(error){
         Serial.print("deserializeJson failed with code");
         Serial.println(error.c_str()); // print the error
      }
      else{
        waterMotorActuator = doc["waterMotorActuator"].as<bool>();
        lightActuator = doc["lightActuator"].as<bool>();
        buzzerActuator = doc["buzzerActuator"].as<bool>();
        fertilizerActuator = doc["fertilizerActuator"].as<bool>();
        // Output to serial monitor
        Serial.print("waterMotorActuator:");
        Serial.println(waterMotorActuator);
        Serial.print("lightActuator:");
        Serial.println(lightActuator);
        Serial.print("buzzerActuator:");
        Serial.println(buzzerActuator);
        Serial.println("fertilizerActuator:");
        Serial.println(fertilizerActuator);

//        if (lightActuator /* && lightIntensity // less than certain value*/) {
//          Serial.println("Switchin on Lights");
//          digitalWrite(growLightPin, HIGH);
//          Serial.println("Lights on");
          // switch on lights
//        }
//        else {
//          digitalWrite(growLightPin, LOW);
//          Serial.println("Lights off");
          // switch off lights
//        }


        // send data
        // soilMoisture
        // humidity
        // temperature
        // lightIntensity
      }
    }
    
    http.end();   //Close connection
 

  }
  else {

  }

//  delay(10000);    //Send a request every 30 seconds
}

TimedAction recieveActuatorStatusThread = TimedAction(10000, recieveActuatorStatus);



void sendSensorData(){
  
  if(WiFi.status()==WL_CONNECTED){ //Check WiFi connection status
    Serial.println("Connected");

    HTTPClient http;  //Declare an object of class HTTPClient

    String soilMoisture = soilMoistureReading ? soilMoistureReading : "";
    String humidity = humidityReading ? humidityReading : "";
    String temperature = temperatureReading ? temperatureReading : "";
    String lightIntensity = lightIntensityReading ? lightIntensityReading : "";

    String postData = "moduleID=" + moduleId + "&soilMoisture=" + soilMoisture +  "&humidity=" + humidity + "&temperature=" + temperature + "&lightIntensity=" + lightIntensity;
//    String postData = "moduleID=5ec66db7aa16ff3a80870c9a&soilMoisture=70.00&humidity=30%&temperature=37C&lightIntensity=200lux";
    Serial.println(postData);
    http.begin("http://192.168.8.104:3002/units/update-data");  //get actuator status - wether the actuators should be activated or not
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
    Serial.println("Failed to connect to WiFi");
  }

//  delay(10000);    //Send a request every 30 seconds
}

TimedAction sendSensorDataThread = TimedAction(10000, sendSensorData);

void sendData(){
  
    StaticJsonDocument<300> nodedoc;
        
    nodedoc["waterMotor"] = waterMotorActuator;
    nodedoc["fertilizerMotor"] = fertilizerActuator;
    nodedoc["light"] = lightActuator;
//    nodedoc["automated"] = automated;
    
//    Serial.println("NodeMCU Sending Data");
//    Serial.print("water state: ");
//    Serial.println(nodedoc["waterMotor"].as<bool>());
  
    // Send the JSON document over the serial port
//    serializeJson(nodedoc, Serial);

//    if(s.available()>0){
      serializeJson(nodedoc, s);
//      Serial.println("Data sent to Mega");
//      nodedoc.printTo(s);
//    }
  //  delay(3000);
}

TimedAction sendDataThread = TimedAction(3000, sendData);


void recieveData(){
  // Check if the Arduino is transmitting
  if (Serial.available()){
    // Allocate the JSON document
    // This one must be bigger than the sender because it must store the strings
    StaticJsonDocument<350> doc;

    // Read the JSON document from the serial port
    DeserializationError err = deserializeJson(doc, Serial);

    if (err == DeserializationError::Ok){


      soilMoistureReading = doc["soilMoisture"].as<String>();
      humidityReading = doc["humidity"].as<String>();
      temperatureReading = doc["temperature"].as<String>();
      lightIntensityReading = doc["lightIntensity"].as<String>();

      
//      waterMotorActuator = doc["waterMotor"].as<bool>();
//      lightActuator = doc["light"].as<bool>();
//      buzzerActuator = doc["buzzer"].as<bool>();
//      fertilizerActuator = doc["fertilizerMotor"].as<bool>();

      
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      Serial.println("Recieving Data from Mega");
      Serial.print("Soil Moisture = ");
      Serial.println(soilMoistureReading);
      Serial.print("Light Intensity = ");
      Serial.println(lightIntensityReading);
      Serial.print("Humidity = ");
      Serial.println(humidityReading);
      Serial.print("Temperature = ");
      Serial.println(temperatureReading);

//      Serial.print("Water motor state = ");
//      Serial.println(waterMotorActuator);
//      Serial.print("Fertilizer motor state = ");
//      Serial.println(fertilizerActuator);
//      Serial.print("Light state = ");
//      Serial.println(lightActuator);
//      Serial.print("Buzzer state = ");
//      Serial.println(buzzerActuator);

    } 
    else{
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());
  
      // Flush all bytes in the serial port buffer
      while (Serial.available() > 0)
        Serial.read();
    }

//    delay(3000);
  }
}

TimedAction recieveDataThread = TimedAction(3000, recieveData);
 
void loop() {

  recieveActuatorStatusThread.check();
//  recieveActuatorStatus();
  recieveDataThread.check();
//  recieveData();
  sendDataThread.check();
//  sendData();
  sendSensorDataThread.check();
//  sendSensorData();
  
}
