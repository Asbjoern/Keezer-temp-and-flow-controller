#include <PersWiFiManager.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <EasySSDP.h>
#include <ESP8266mDNS.h>
#include <SPIFFSReadServer.h>
// upload data folder to chip with Arduino ESP8266 filesystem uploader
// https://github.com/esp8266/arduino-esp8266fs-plugin
#include <DNSServer.h>
#include <FS.h>  
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include "DS18B20Events.h"

#define DEBUG_BEGIN Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY, 1);
#define DEBUG_PRINT(x) Serial.print(x);Log.push_back(x);sendmsg(String(mqtt_topic)+"/status",x);
#define DEBUG_PRINTLN(x) Serial.println(x);Log.push_back(x);sendmsg(String(mqtt_topic)+"/status",x);

#define ONE_WIRE_BUS 0
#define FRIDGE 2
#define PUMP 15
const uint8_t flowPin[6] = {3,4,5,12,13,14};

char mqtt_server[40] = "";
char mqtt_port[6] = "1883";
char mqtt_uid[40];
char mqtt_pwd[40];
char mqtt_topic[40]="keezer";
char hostname[40] ="keezer";
float setpoint = 3.3;
float temp = 3.3;
float threshold = 0.5;
int cntprlitre[6] = {2520,2520,2520,2520,2520,2520}; ////(21*60)*2.
float kegContentLitre[6] = {0,0,0,0,0,0};
bool isAP=true;

#define LOGSIZE 50
std::vector<String> Log;

SPIFFSReadServer server(80);
DNSServer dnsServer;
PersWiFiManager persWM(server, dnsServer);
WiFiClient espClient;
PubSubClient psclient(espClient);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemperature(&oneWire);
DS18B20Events thermometer(0);

Ticker saveTicker;
Ticker publishTicker;

void setup(void){
  DS18B20Events::setup(&dallasTemperature); 
  DS18B20Events::setInterval(5000); 
  thermometer.onChange = temperatureChanged;
  
  pinMode(flowPin[0], INPUT);
  pinMode(flowPin[1], INPUT);
  pinMode(flowPin[2], INPUT);
  pinMode(flowPin[3], INPUT);
  pinMode(flowPin[4], INPUT);
  pinMode(flowPin[5], INPUT);
  pinMode(FRIDGE, OUTPUT);
  digitalWrite(FRIDGE,HIGH);
  attachInterrupt(digitalPinToInterrupt(flowPin[0]), flow0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(flowPin[1]), flow1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(flowPin[2]), flow2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(flowPin[3]), flow3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(flowPin[4]), flow4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(flowPin[5]), flow5, CHANGE);

  DEBUG_BEGIN
  DEBUG_PRINTLN("")
  if (SPIFFS.begin()) 
    DEBUG_PRINTLN("Mounted file system");
  loadConfig();

  WiFi.hostname(hostname);
  persWM.onConnect(&on_connect);
  persWM.onAp([](){
    DEBUG_PRINTLN("AP MODE: " + persWM.getApSsid());
    isAP=true;
  });
  persWM.setApCredentials(hostname);
  persWM.setConnectNonBlock(false);
  persWM.begin();
  
  server.on("/log", []() {
    String logs = "Log contains:\n";
    for (int i = 0; i<Log.size();i++)
      logs += Log.at(i) + '\n';
    server.send(200, "text/plain", logs.c_str());
   });
  server.on("/config", []() {
    DEBUG_PRINTLN("server.on /config");
    if (server.hasArg("mqtt_server")) {
      strlcpy(mqtt_server,server.arg("mqtt_server").c_str(),sizeof(mqtt_server)-1);
      DEBUG_PRINTLN("mqtt_server: " + server.arg("mqtt_server"));
    } 
    if (server.hasArg("mqtt_port")) {
      strlcpy(mqtt_port,server.arg("mqtt_port").c_str(),sizeof(mqtt_port)-1);
      DEBUG_PRINTLN("mqtt_port: " + server.arg("mqtt_port"));
    }
    if (server.hasArg("mqtt_uid")) {
      strlcpy(mqtt_uid,server.arg("mqtt_uid").c_str(),sizeof(mqtt_uid)-1);
      DEBUG_PRINTLN("mqtt_uid: " + server.arg("mqtt_uid"));
    } 
    if (server.hasArg("mqtt_pwd")) {
      strlcpy(mqtt_pwd,server.arg("mqtt_pwd").c_str(),sizeof(mqtt_pwd)-1);
      DEBUG_PRINTLN("mqtt_pwd: " + server.arg("mqtt_pwd"));
    } 
    if (server.hasArg("mqtt_topic")) {
      strlcpy(mqtt_topic,server.arg("mqtt_topic").c_str(),sizeof(mqtt_topic)-1);
      DEBUG_PRINTLN("mqtt_topic: " + server.arg("mqtt_topic"));
    } 
    if (server.hasArg("hostname")) {
      strlcpy(hostname,server.arg("hostname").c_str(),sizeof(hostname)-1);
      DEBUG_PRINTLN("hostname: " + server.arg("hostname"));
    }
    if (server.hasArg("setpoint")) {
      setpoint = server.arg("setpoint").toFloat();
      DEBUG_PRINTLN("setpoint: " + server.arg("setpoint"));
    }
    if (server.hasArg("threshold")) {
      setpoint = server.arg("threshold").toFloat();
      DEBUG_PRINTLN("threshold: " + server.arg("threshold"));
    }
    if (server.hasArg("cntprlitre0")) {
      cntprlitre[0] = server.arg("cntprlitre0").toInt();
      DEBUG_PRINTLN("cntprlitre0: " + server.arg("cntprlitre0"));
    }
    if (server.hasArg("cntprlitre1")) {
      cntprlitre[1] = server.arg("cntprlitre1").toInt();
      DEBUG_PRINTLN("cntprlitre1: " + server.arg("cntprlitre1"));
    }
    if (server.hasArg("cntprlitre2")) {
      cntprlitre[2] = server.arg("cntprlitre2").toInt();
      DEBUG_PRINTLN("cntprlitre2: " + server.arg("cntprlitre2"));
    }
    if (server.hasArg("cntprlitre3")) {
      cntprlitre[3] = server.arg("cntprlitre3").toInt();
      DEBUG_PRINTLN("cntprlitre3: " + server.arg("cntprlitre3"));
    }
    if (server.hasArg("cntprlitre4")) {
      cntprlitre[4] = server.arg("cntprlitre4").toInt();
      DEBUG_PRINTLN("cntprlitre4: " + server.arg("cntprlitre4"));
    }
    if (server.hasArg("cntprlitre5")) {
      cntprlitre[5] = server.arg("cntprlitre5").toInt();
      DEBUG_PRINTLN("cntprlitre5: " + server.arg("cntprlitre5"));
    }
    if (server.hasArg("kegContentLitre0")) {
      kegContentLitre[0] = server.arg("kegContentLitre0").toFloat();
      DEBUG_PRINTLN("kegContentLitre0: " + server.arg("kegContentLitre0"));
    }
    if (server.hasArg("kegContentLitre1")) {
      kegContentLitre[1] = server.arg("kegContentLitre1").toFloat();
      DEBUG_PRINTLN("kegContentLitre1: " + server.arg("kegContentLitre1"));
    }
    if (server.hasArg("kegContentLitre2")) {
      kegContentLitre[2] = server.arg("kegContentLitre2").toFloat();
      DEBUG_PRINTLN("kegContentLitre2: " + server.arg("kegContentLitre2"));
    }
    if (server.hasArg("kegContentLitre3")) {
      kegContentLitre[3] = server.arg("kegContentLitre3").toFloat();
      DEBUG_PRINTLN("kegContentLitre3: " + server.arg("kegContentLitre3"));
    }
    if (server.hasArg("kegContentLitre4")) {
      kegContentLitre[4] = server.arg("kegContentLitre4").toFloat();
      DEBUG_PRINTLN("kegContentLitre4: " + server.arg("kegContentLitre4"));
    }
    if (server.hasArg("kegContentLitre5")) {
      kegContentLitre[5] = server.arg("kegContentLitre5").toFloat();
      DEBUG_PRINTLN("kegContentLitre5: " + server.arg("kegContentLitre5"));
    }

    if(server.args() > 2){
       saveConfig();
    }

  StaticJsonDocument<1024> json;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_uid"] = mqtt_uid;
  json["mqtt_pwd"] = mqtt_pwd;
  json["mqtt_topic"] = mqtt_topic;
  json["hostname"] = hostname;
  json["setpoint"] = setpoint;
  json["threshold"] = threshold;
  JsonArray arrCntprlitre = json.createNestedArray("cntprlitre");
  JsonArray arrKegContentLitre = json.createNestedArray("kegContentLitre");
  for(int i = 0; i<6; i++){
    arrCntprlitre.add(cntprlitre[i]);
    arrKegContentLitre.add(kegContentLitre[i]);
  }
  String tmp;
  serializeJson(json, tmp);
    server.send(200, "application/json", tmp);
  });
  server.begin();
  publishTicker.attach(30, publishReadings);
  DEBUG_PRINTLN("Setup complete.");
  
}

void loop(void){  

  thermometer.loop();
  if(Log.size()>LOGSIZE){
    std::rotate(Log.begin(),Log.end()-LOGSIZE,Log.end());
    Log.resize(LOGSIZE);
  }
  persWM.handleWiFi();
  dnsServer.processNextRequest();
  server.handleClient();
  MDNS.update();

  if (!psclient.connected() && !isAP && mqtt_server[0]!='\0') {
    mqttReconnect();
    for(auto msg : Log)
      sendmsg(String(mqtt_topic)+"/status",msg);
  }
  psclient.loop();
  
  if(temp > setpoint + threshold)
    digitalWrite(FRIDGE,LOW);
  else if(temp < setpoint - threshold)
    digitalWrite(FRIDGE,HIGH);
    
}

void temperatureChanged(uint8_t index, float tempC)
{
  temp = tempC;
  //DEBUG_PRINTLN(String(tempC) + " new tempC at index " + index);
}

ICACHE_RAM_ATTR void flow0() {kegContentLitre[0]-= 1./cntprlitre[0];saveTicker.once(30,saveConfig);}
ICACHE_RAM_ATTR void flow1() {kegContentLitre[1]-= 1./cntprlitre[1];saveTicker.once(30,saveConfig);}
ICACHE_RAM_ATTR void flow2() {kegContentLitre[2]-= 1./cntprlitre[2];saveTicker.once(30,saveConfig);}
ICACHE_RAM_ATTR void flow3() {kegContentLitre[3]-= 1./cntprlitre[3];saveTicker.once(30,saveConfig);}
ICACHE_RAM_ATTR void flow4() {kegContentLitre[4]-= 1./cntprlitre[4];saveTicker.once(30,saveConfig);}
ICACHE_RAM_ATTR void flow5() {kegContentLitre[5]-= 1./cntprlitre[5];saveTicker.once(30,saveConfig);}


void on_connect() {
  DEBUG_PRINTLN("Wifi connected");
  DEBUG_PRINTLN("Connect to http://"+String(hostname)+".local or http://" + WiFi.localIP().toString());
  isAP = false;
  EasySSDP::begin(server);
  if (MDNS.begin(hostname)) {
    DEBUG_PRINTLN("MDNS responder started");
  } else {
    DEBUG_PRINTLN("Error setting up MDNS responder!");
  }
  MDNS.addService("http", "tcp", 80);
  DEBUG_PRINTLN("Connecting to MQTT");
  psclient.setServer(mqtt_server, atoi(mqtt_port));
  psclient.setCallback(mqtt_callback);
}

void loadConfig(){
  DEBUG_PRINTLN("Load config");
  if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      DEBUG_PRINTLN("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        DEBUG_PRINTLN("opened config file");
        size_t size = configFile.size();
        StaticJsonDocument<1024> doc;
        DeserializationError error = deserializeJson(doc,configFile);
        if (error){
          DEBUG_PRINTLN("Failed to read file. Using default configuration. - "+  String(error.c_str()));
        }
        else
        {
          strcpy(mqtt_server, doc["mqtt_server"]);
          strcpy(mqtt_port, doc["mqtt_port"]);
          strcpy(mqtt_uid, doc["mqtt_uid"]);
          strcpy(mqtt_pwd, doc["mqtt_pwd"]);
          strcpy(mqtt_topic, doc["mqtt_topic"]);
          strcpy(hostname, doc["hostname"]);
          setpoint = doc["setpoint"].as<float>();
          threshold = doc["threshold"].as<float>();;
          JsonArray arrCntprlitre = doc["cntprlitre"].as<JsonArray>();
          JsonArray arrKegContentLitre = doc["kegContentLitre"].as<JsonArray>();
          for(int i=0; i<6; i++)
          {
            cntprlitre[i] = arrCntprlitre[i].as<int>();
            kegContentLitre[i] = arrKegContentLitre[i].as<float>();
          }
        }
        configFile.close();
        serializeJson(doc, Serial);
      }
  }
  else{
    DEBUG_PRINTLN("Config file not found. Using default configuration");
  }
  DEBUG_PRINTLN("MQTT server: " + String(mqtt_server));
  DEBUG_PRINTLN("MQTT port: " + String(mqtt_port));
  DEBUG_PRINTLN("MQTT user: " + String(mqtt_uid));
  DEBUG_PRINTLN("MQTT password: " + String(mqtt_pwd));
  DEBUG_PRINTLN("MQTT topic: " + String(mqtt_topic));
  DEBUG_PRINTLN("Hostname: " + String(hostname));
}

void saveConfig(){
  DEBUG_PRINTLN("saving config");
    
  StaticJsonDocument<1024> json;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_uid"] = mqtt_uid;
  json["mqtt_pwd"] = mqtt_pwd;
  json["mqtt_topic"] = mqtt_topic;
  json["hostname"] = hostname;
  json["setpoint"] = setpoint;
  json["threshold"] = threshold;
  JsonArray arrCntprlitre = json.createNestedArray("cntprlitre");
  JsonArray arrKegContentLitre = json.createNestedArray("kegContentLitre");
  for(int i = 0; i<6; i++){
    arrCntprlitre.add(cntprlitre[i]);
    arrKegContentLitre.add(kegContentLitre[i]);
  }

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    DEBUG_PRINTLN("failed to open config file for writing");
  }
  serializeJson(json, configFile);
  configFile.close();
}

void mqttReconnect() {
  if (!psclient.connected()) {
    DEBUG_PRINTLN("Attempting MQTT connection...");
    if (psclient.connect(hostname)) {
      psclient.subscribe((String(mqtt_topic) + "/set/+").c_str());
      DEBUG_PRINTLN("connected " + String(mqtt_topic));
    } else {
      DEBUG_PRINTLN("failed, rc=" + String(psclient.state()));
    }
  }
}

void sendmsg(String topic, String payload) {
  if(psclient.connected()){
    psclient.publish(topic.c_str(),payload.c_str());
  }
}

void publishReadings(){
  DEBUG_PRINTLN("publish");
  sendmsg(String(mqtt_topic)+"/data/temp",String(temp));
  sendmsg(String(mqtt_topic)+"/data/setpoint",String(setpoint));
  sendmsg(String(mqtt_topic)+"/data/threshold",String(threshold));
  for(int i = 0; i<6; i++){
    //sendmsg(String(mqtt_topic)+"/data/cntprlitre" + String(i),String(cntprlitre[i]));
    sendmsg(String(mqtt_topic)+"/data/kegContentLitre" + String(i),String(kegContentLitre[i]));
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {

  payload[length] = '\0';
  String topicStr = String(topic);
  String s = String((char*)payload);
  float f = s.toFloat();
  Serial.println(topicStr);
  if (topicStr == String(mqtt_topic)+"/set/setpoint")
    setpoint = f;
  else if(topicStr == String(mqtt_topic)+"/set/threshold")
    threshold = f;
  else if(topicStr == String(mqtt_topic)+"/set/kegContentLitre0")
    kegContentLitre[0] = f;
  else if(topicStr == String(mqtt_topic)+"/set/kegContentLitre1")
    kegContentLitre[1] = f;
  else if(topicStr == String(mqtt_topic)+"/set/kegContentLitre2")
    kegContentLitre[2] = f;
  else if(topicStr == String(mqtt_topic)+"/set/kegContentLitre3")
    kegContentLitre[3] = f;
  else if(topicStr == String(mqtt_topic)+"/set/kegContentLitre4")
    kegContentLitre[4] = f;
  else if(topicStr == String(mqtt_topic)+"/set/kegContentLitre5")
    kegContentLitre[5] = f;

  saveConfig();
}
