#include <StreamUtils.h>

#define ARDUINOJSON_ENABLE_COMMENTS 1
#include <ArduinoJson.h>

#include <TimedAction.h>
#include <dht.h>

dht DHT;
#define DHT11_PIN 7

int ledPin = 13;                // LED 
int pirPin = 2;                 // PIR Out pin 
int pirStat = 0;                   // PIR status

bool waterMotorState = false;
bool fertilizerMotorState = false;
bool lightState = false;
bool buzzerState = false;
bool automated = true;


int soilMoisture = 0;
int lightIntensity = 0;
float humidity = 0;
float temperature = 0;
bool motionDetected = false;


//int soilMoisture = 0;
//float humidity = 0;


void motionDetection(){
  pirStat = digitalRead(pirPin); 
  if(automated){
    if (pirStat == HIGH) {            // if motion detected
      motionDetected = true;
      digitalWrite(ledPin, HIGH);  // turn LED ON
      Serial.println("Motion Detected");
    } 
    else {
      motionDetected = false;
      digitalWrite(ledPin, LOW); // turn LED OFF if we have no motion
    }
  }  
}

TimedAction motionThread = TimedAction(100, motionDetection);

void humidityDetection(){
  int chk = DHT.read11(DHT11_PIN);
  
  humidity = DHT.humidity;
  temperature = DHT.temperature;
  
  Serial.print("Temperature = ");
  Serial.println(temperature);
  Serial.print("Humidity = ");
  Serial.println(humidity);
//  delay(1000);
}

TimedAction humidityThread = TimedAction(1000, humidityDetection);

void setup() {
  // Initialize "debug" serial port
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);     
  pinMode(pirPin, INPUT);
  
  while (!Serial) continue;

  Serial3.begin(9600);
  Serial2.begin(9600);
}

void getSensorData(){
  // water moisture
  // humidity
  // temperature
  // light intensity
  // motion detector
//  motionDetection();

  if(motionDetected) Serial.println("Motion");

  soilMoisture = int(random(90, 100));
  lightIntensity = int(random(0, 1000));
  humidity = int(random(10, 90));
  temperature = int(random(10, 50));
}

TimedAction getSensorDataThread = TimedAction(100, getSensorData);

void controlActuators(){
  // water plants
  // swich on grow lights
  // add fertilizer
  
  
  long timestamp = millis();

  waterMotorState = (timestamp%2==0) ? true : false;
  fertilizerMotorState = (timestamp%3==0) ? true : false;
  lightState = (timestamp%5==0) ? true : false;
  buzzerState = (timestamp%7==0) ? true : false;
}

void sendData(){
    
//  long timestamp = millis();
  

  // Print the values on the "debug" serial port
//  Serial.print("timestamp = ");
//  Serial.println(timestamp);

  Serial.println("Sending data to NodeMCU");
  Serial.print("Soil Moisture = ");
  Serial.println(soilMoisture);
  Serial.print("Light Intensity = ");
  Serial.println(lightIntensity);
  Serial.print("Humidity = ");
  Serial.println(humidity);
  Serial.print("Temperature = ");
  Serial.println(temperature);

  // Create the JSON document
  StaticJsonDocument<250> megadoc;
//  megadoc["timestamp"] = timestamp;
  megadoc["soilMoisture"] = soilMoisture;
  megadoc["lightIntensity"] = lightIntensity;
  megadoc["humidity"] = humidity;
  megadoc["temperature"] = temperature;
  megadoc["waterMotor"] = waterMotorState;
  megadoc["fertilizerMotor"] = fertilizerMotorState;
  megadoc["light"] = lightState;
  megadoc["buzzer"] = buzzerState;

  // Send the JSON document over the "link" serial port
  serializeJson(megadoc, Serial3);

  // Wait
//  delay(3000);
}

TimedAction sendDataThread = TimedAction(3000, sendData);

void recieveData(){
  // Check if the nodemuc is transmitting
//  Serial.println("Checking serial data");
  if (Serial2.available()){

//    Serial.println("serial available");
    // Allocate the JSON document
    // This one must be bigger than for the sender because it must store the strings
    
    StaticJsonDocument<450> doc;
//    StaticJsonDocument<350> doc;

    // Read the JSON document from the serial port

//    Serial2.setTimeout(10000);

    ReadLoggingStream loggingStream(Serial2, Serial);
    DeserializationError err = deserializeJson(doc, loggingStream);

    if (err == DeserializationError::Ok){

//      Serial.println("No serial error");
      
      waterMotorState = doc["waterMotor"].as<bool>();
      fertilizerMotorState = doc["fertilizerMotor"].as<bool>();
      lightState = doc["light"].as<bool>();
//      automated = doc["automated"].as<bool>();
      
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      Serial.println("Recieving data from NodeMCU");
      Serial.print("water motor = ");
      Serial.println(waterMotorState);
      Serial.print("light = ");
      Serial.println(lightState);
      Serial.print("fertilizer motor = ");
      Serial.println(fertilizerMotorState);
      
    } 
    else{
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());
  
      // Flush all bytes in the "link" serial port buffer
      while (Serial2.available() > 0)
        Serial2.read();
    }
  }
//  delay(3000);
}

TimedAction recieveDataThread = TimedAction(3000, recieveData);

void loop() {

  motionThread.check();
  humidityThread.check();
  recieveDataThread.check();
//  recieveData();
  getSensorDataThread.check();
//  getSensorData();
//  controlActuators();
  sendDataThread.check();
//  sendData();
  
// 
//    delay(3000);
//
    
//
    
//
//    sendData();
//  }
//  
//  else{
//   sendData();
//  }
}
