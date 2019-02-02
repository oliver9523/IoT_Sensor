/*

IoT Sensor for Home Automation System

MQTT Topics
devices/poke = prompts all devices to report their state
devices/update = allows the device to receive OTA updates if the message matches its MAC

pub_motion = sends 'alert' if motion is detected
pub_temperature = send temperature values

WTFPL license

If this helps you in any way feel free to buy me a drink
https://www.buymeacoffee.com/manythanks

*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <dht.h>

//struct
//{
//    uint32_t total;
//    uint32_t ok;
//    uint32_t crc_error;
//    uint32_t time_out;
//    uint32_t connect;
//    uint32_t ack_l;
//    uint32_t ack_h;
//    uint32_t unknown;
//} stat = { 0,0,0,0,0,0,0,0};

//==============================
//    HARDWARE SETTINGS
//==============================
//#define DHTPIN 4     // what digital pin the DHT22 is conected to
//#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors

#define DHT22_PIN 4

const int LED = D4;         // LED pin
const int TEMP_PIN = A0;    // TEMP pin
const int PIR_PIN = D1;     // PIR pin - SAFE to be LOW or HIGH on BOOT on D1 MINI

//==============================
//    NETWORK SETTINGS
//==============================
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* mqtt_server = "MQTT_IP";

//==============================
//    GLOBAL VARIABLES
//==============================
WiFiClient espClient;
PubSubClient client(espClient);
String JSON;

StaticJsonBuffer<500> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();

StaticJsonBuffer<500> jsonErrorBuffer;
JsonObject& error_report = jsonErrorBuffer.createObject();

//DHT dht(DHTPIN, DHTTYPE);
dht DHT;



String MAC;
String IP;


//      DEVICE LOCATION
//==============================
String FLOOR;
String ROOM;
String TYPE;
String LOCATION;

//      MQTT TOPICS
//==============================
String sub_devices_poke = "devices/poke";
String sub_devices_update = "devices/update";
String pub_devices_error = "devices/error";
String pub_devices = "devices";

String pub_motion;
String pub_temperature;
String pub_humidity;
String Firmware = "1.2";

bool motion_published = false;
int prev_temp = -999;

unsigned long temp_lastpubtime = 0;
unsigned long humi_lastpubtime = 0;
unsigned long sensor_pubrate = 60000;

bool Allow_OTA = false;
unsigned long OTA_Timeout = 60000;
unsigned long OTA_Timer = 0;

unsigned long timeSinceLastTempHumiRead = 0;

//==============================
//    SETUP FUNCTIONS
//==============================
void setup() {
  Serial.begin(115200);
  SetupPins();
  SetupWiFi();
  SetupOTA();
  SetupMQTT();

  SetLocation("0","hall","sensors");
  GetDeviceInfo();
  reconnect();
}

void SetupPins(){
  pinMode(LED, OUTPUT);
  pinMode (TEMP_PIN, INPUT);
  pinMode (PIR_PIN, INPUT_PULLUP);
  digitalWrite(LED, HIGH);
  Serial.println("PINS Ready");
}

void SetupWiFi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void SetupOTA(){

  ArduinoOTA.setPassword((const char *)"OTA_PASSWORD");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

void SetupMQTT(){
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.println("MQTT Ready");
}

void AllowOTA(){
  Allow_OTA = true;
  OTA_Timer = millis();
}

void publishDevice(){
  //just randomly delay between 1-1000ms to avoid 
  //everything hitting the broker at the same time
  delay(random(1,1000));
  client.publish(pub_devices.c_str(), JSON.c_str());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MAC.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //publish the device info to the device channel
      publishDevice();
      
      // ... and resubscribe
      client.subscribe(sub_devices_poke.c_str());
      client.subscribe(sub_devices_update.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void SetLocation(String Floor, String Room, String Type){
  //Set the location of the device
  FLOOR = Floor;
  ROOM = Room;
  TYPE = Type;
  LOCATION = FLOOR + "/" + ROOM + "/" + TYPE + "/";

  BuildPubTopics();
}

void BuildPubTopics(){
  //Allow us to set location 
  pub_motion = LOCATION + "motion";
  pub_temperature = LOCATION + "temp";
  pub_humidity = LOCATION + "humidity";
}

//==============================
//    DEVICES SPECIFIC Fn
//==============================

void callback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == sub_devices_poke){
    //don't care what the message is
    publishDevice();
  } else if (String(topic) == sub_devices_update){
    if (!strncmp((char *)payload, MAC.c_str(), length)) {
      AllowOTA();
    }
  }
}

void GetDeviceInfo(){
  byte mac[6];
  WiFi.macAddress(mac);
  char mac_[20];
  sprintf(mac_, "%2X%2X%2X%2X%2X%2X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  //network and device info
  IP = WiFi.localIP().toString();
  MAC = String(mac_);
  
  //populate JSON with device info
  root["description"] = "multi-sensor";
  root["ip"] = IP.c_str();
  root["mac"] = MAC.c_str();
  root["version"] = Firmware.c_str();

  //populate JSON with description of the data the device sends
  JsonObject& f0 = root.createNestedObject("function0");
  f0["type"] = "temperature sensor";
  f0["value"] = "celsius";
  f0["control"] = "pub";
  f0["topic"] = pub_temperature.c_str();

  JsonObject& f1 = root.createNestedObject("function1");
  f1["type"] = "humidity sensor";
  f1["value"] = "%";
  f1["control"] = "pub";
  f1["topic"] = pub_humidity.c_str();

  JsonObject& f2 = root.createNestedObject("function2");
  f2["type"] = "motion sensor";
  f2["value"] = "alert";
  f2["control"] = "pub";
  f2["topic"] = pub_motion.c_str();

  root.printTo(JSON);
}

void publishError(String error){
  //populate JSON with device info
  error_report["description"] = "multi-sensor";
  error_report["ip"] = IP.c_str();
  error_report["mac"] = MAC.c_str();
  error_report["location"] = LOCATION.c_str();
  error_report["error"] = error.c_str();

  String ErrorJson;
  error_report.printTo(ErrorJson);
  client.publish(pub_devices_error.c_str(), ErrorJson.c_str());
  
}

bool MeasureEnvironment(unsigned long *_time, float *temp, float *humidity){
  if((millis() - *_time) > 10000) {
    //*humidity = dht.readHumidity();
    //*temp = dht.readTemperature();
    //delay(10);
    //if (isnan(*humidity) || isnan(*temp)) {
    //  Serial.println("Failed to read from DHT sensor!");
    //  *_time = 0;
    //  return false;
    //}

    int chk = DHT.read22(DHT22_PIN);
    
    //stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        //stat.ok++;
        Serial.println("OK\t");
        *temp = DHT.temperature;
        *humidity = DHT.humidity;
        *_time = millis();
        return true;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        //stat.crc_error++;
        Serial.println("Checksum error\t");
        publishError("DHT22 Checksum error");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        //stat.time_out++;
        Serial.println("Time out error\t");
        publishError("DHT22 Time out error");
        break;
    default:
        //stat.unknown++;
        Serial.println("Unknown error\t");
        publishError("DHT22 Unknown error");
        break;
    }
    *_time = millis();
    return false;
  }
  return false;
}

float readHumidity(){
  return -1;
}

void publishHumidity(){
  if(millis() - humi_lastpubtime > sensor_pubrate) {
    humi_lastpubtime = millis();
    int current_humidity = (int)readHumidity();
    client.publish(pub_humidity.c_str(), String(current_humidity).c_str(), true);
  }
}

float readTemp(){
  int analogValue = analogRead(TEMP_PIN);
  float millivolts = (analogValue/1024.0) * 3300; //3300 is the voltage provided by NodeMCU
  //float celsius = millivolts/10;
  float celsius = (millivolts - 500) / 10 ;
  Serial.print("in DegreeC=   ");
  Serial.println(celsius);
  return celsius;
}

void publishTemp(){
  if(millis() - temp_lastpubtime > sensor_pubrate) {
    temp_lastpubtime = millis();
    int current_temp = (int)readTemp();
    client.publish(pub_temperature.c_str(), String(current_temp).c_str(), true);
  }
}

void publishEnviromentalData(){
  float temp;
  float humi;
    if (MeasureEnvironment(&timeSinceLastTempHumiRead, &temp, &humi)){
      client.publish(pub_temperature.c_str(), String(temp).c_str(), true);
      client.publish(pub_humidity.c_str(), String(humi).c_str(), true);
    }
}

void publishMotion(){

  //Read PIR input
  int PIR_VAL = digitalRead(PIR_PIN);
  //PIR_VAL == 1 is Open circuit (pull up) = alarm
  //PIR_VAL == 0 is Short circuit = normal

  if (PIR_VAL == 1) {
    //Open circuit = alarm
    digitalWrite(LED, LOW);   //inverted logic for wemos
    if (!motion_published){
      client.publish(pub_motion.c_str(), "alert");
      motion_published = true;
    }
  }else{
    //Closed circuit = safe
    digitalWrite(LED, HIGH);  //inverted logic for wemos
    motion_published = false;
  }

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }

  //Perform MQTT loop
  client.loop();

  publishEnviromentalData();    //publish temp humi every 10s
  publishMotion();              //publish motion changes

  //Enable OTA for 60s
  if (Allow_OTA){
    uint16_t elapsed = 0;
    while (elapsed < OTA_Timeout){
      ArduinoOTA.handle();
      elapsed = millis() - OTA_Timer;
      delay(10);
    }
    Allow_OTA = false;
  }

}


