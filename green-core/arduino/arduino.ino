#include <StreamUtils.h>

#define ARDUINOJSON_ENABLE_COMMENTS 1
#include <ArduinoJson.h>

// light intensity sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#include <TimedAction.h>
#include <dht.h>

dht DHT;
#define DHT11_PIN 8

int buzzerPin = 13;                // LED
int pirPin = 10;                 // PIR Out pin
int growlightPin = 9;           // Grow Light pin

int EN_A = 7;      //Enable pin for first motor
int IN1 = 6;       //control pin for first motor
int IN2 = 5;       //control pin for first motor
int IN3 = 4;        //control pin for second motor
int IN4 = 3;        //control pin for second motor
int EN_B = 2;      //Enable pin for second motor


int pirStat = 0;                   // PIR status

bool waterMotorState = false;
bool fertilizerMotorState = false;
bool lightState = false;
bool buzzerState = false;
bool automated = false;


int soilMoisture = 0;
float lightIntensity = 0;
float humidity = 0;
float temperature = 0;
bool motionDetected = false;


//int soilMoisture = 0;
//float humidity = 0;


// soil moisture

void soilMoistureSensor(){
  soilMoisture = analogRead(A0);
  Serial.print(soilMoisture);
  Serial.print(" - ");
  if(soilMoisture >= 1000){
    Serial.println("Sensor is not in the Soil or DISCONNECTED");
  }
  if(soilMoisture < 1000 && soilMoisture >= 600){ 
    Serial.println("Soil is DRY");
  }

  if(soilMoisture < 600 && soilMoisture >= 370){
    Serial.println("Soil is HUMID"); 
  }
  
  if(soilMoisture < 370){
    Serial.println("Sensor in WATER");
  }                                          

}

TimedAction soilMoistureThread = TimedAction(2000, soilMoistureSensor);



// light intensity
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void configureLightSensor(void){
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}


void lightIntensitySensor(){
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  if (event.light){
    lightIntensity = event.light;
  }
  else{
    Serial.println("Light Sensor overload");
  }
}

TimedAction lightIntensityThread = TimedAction(250, lightIntensitySensor);

void growLight() {
  if(automated){
    if(lightIntensity < 70){
      digitalWrite(growlightPin, HIGH);
    }
    else{
      digitalWrite(growlightPin, LOW);
    }
  }
  else{               // manual configuration
    if(lightState){
      digitalWrite(growlightPin, HIGH);
    }
    else{
      digitalWrite(growlightPin, LOW);
    }
  }
}

TimedAction growLightThread = TimedAction(100, growLight);


void waterMotor() {
  if(automated){
    if(soilMoisture < 1000 && soilMoisture >= 600){
      Serial.println("Watering plants");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(EN_A, 255);
    }
    else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(EN_A, 0);
    }
  }
  else{
    if (waterMotorState) {
      Serial.println("Watering plants");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(EN_A, 255);
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(EN_A, 0);
    }
  }
}

TimedAction waterMotorThread = TimedAction(1000, waterMotor);

void fertilizerMotor() {
  if (fertilizerMotorState) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN_B, 255);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN_B, 0);
  }
}

TimedAction fertilizerMotorThread = TimedAction(1000, fertilizerMotor);

void motionDetection() {
  pirStat = digitalRead(pirPin);
  if (automated) {
    if (pirStat == HIGH) {            // if motion detected
      motionDetected = true;
      digitalWrite(buzzerPin, HIGH);  // turn LED ON
      Serial.println("Motion Detected");
    }
    else {
      motionDetected = false;
      digitalWrite(buzzerPin, LOW); // turn LED OFF if we have no motion
    }
  }
  else{
    digitalWrite(buzzerPin, LOW); // turn LED OFF if we have no motion
  }
}

TimedAction motionThread = TimedAction(1000, motionDetection);

void humidityDetection() {
  int chk = DHT.read11(DHT11_PIN);

  humidity = (DHT.humidity > 0) ? DHT.humidity : humidity; 
  temperature = (DHT.temperature > 0) ? DHT.temperature : temperature;

  //  Serial.print("Temperature = ");
  //  Serial.println(temperature);
  //  Serial.print("Humidity = ");
  //  Serial.println(humidity);
  //  delay(1000);
}

TimedAction humidityThread = TimedAction(1000, humidityDetection);

void setup() {
  // Initialize "debug" serial port
  Serial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  pinMode(pirPin, INPUT);

  pinMode(growlightPin, OUTPUT);

  // soil moisture
  pinMode(A1, INPUT);

  // light intensity
  if(tsl.begin()) configureLightSensor();

  // Motors
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);

  while (!Serial) continue;

  Serial3.begin(9600);
  Serial2.begin(9600);
}

void getSensorData() {
  // water moisture
  // humidity
  // temperature
  // light intensity
  // motion detector
  //  motionDetection();


//  soilMoisture = int(random(90, 100));
//  lightIntensity = int(random(0, 1000));
//  humidity = int(random(10, 90));
//  temperature = int(random(10, 50));
}

TimedAction getSensorDataThread = TimedAction(100, getSensorData);

void controlActuators() {
  // water plants
  // swich on grow lights
  // add fertilizer


  long timestamp = millis();

  waterMotorState = (timestamp % 2 == 0) ? true : false;
  fertilizerMotorState = (timestamp % 3 == 0) ? true : false;
  lightState = (timestamp % 5 == 0) ? true : false;
  buzzerState = (timestamp % 7 == 0) ? true : false;
}

void sendData() {

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

void recieveData() {
  // Check if the nodemuc is transmitting
  //  Serial.println("Checking serial data");
  if (Serial2.available()) {

    //    Serial.println("serial available");
    // Allocate the JSON document
    // This one must be bigger than for the sender because it must store the strings

    StaticJsonDocument<450> doc;
    //    StaticJsonDocument<350> doc;

    // Read the JSON document from the serial port

    //    Serial2.setTimeout(10000);

    ReadLoggingStream loggingStream(Serial2, Serial);
    DeserializationError err = deserializeJson(doc, loggingStream);

    if (err == DeserializationError::Ok) {

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
    else {
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

  soilMoistureThread.check();
  lightIntensityThread.check();
  motionThread.check();
  humidityThread.check();
  recieveDataThread.check();
  //  recieveData();
//  getSensorDataThread.check();
  //  getSensorData();
  growLightThread.check();
  waterMotorThread.check();
  fertilizerMotorThread.check();
  //  controlActuators();
  sendDataThread.check();
  //  sendData();

}
