#include <Arduino.h>
//
//
//
//
//
//
// add MAX17048 batter fuel gauge sensor for 
// precise batter measurement
//
//
//
//
//
//
//
//



//#define DEBUG
#define DO_DEEPSLEEP
#define DO_PUBLISH
#ifdef DO_PUBLISH
#ifdef DEBUG
#define DEEPSLEEPDURATION 10*1000*1000 // 10 seconds
#else
#define DEEPSLEEPDURATION 15*60*1000*1000 // 15 minutes
#endif
#else
#define DEEPSLEEPDURATION 5*1000*1000 // 5 seconds
#endif
#define REBOOTDURATION 5*1000*1000

bool do_publish = true;
bool bmp_init = false;
bool max_init = false;
bool ina_init = false;

#ifdef ESP32
  #include <ESPAsyncWebServer.h>
  #define MANAGER_PIN 23
  #define RESET_PIN   19
  #define ADC_PIN A0
  #define ADC_RESOLUTION 4096.0
  #define ADC_MAX_VOLTAGE 3.3
  #define U_DEVIDER 2.0
#elif defined( ESP8266 )
  // Including the ESP8266 WiFi library
  #include <ESP8266WiFi.h>
  #include <ESPAsyncWebServer.h>
  extern "C" {
    #include "gpio.h"
  }
  extern "C" {
    #include "user_interface.h"
  }
  #define MANAGER_PIN 14
  #define RESET_PIN   12
  #define ADC_PIN A0
  #define ADC_RESOLUTION 1024.0
  #define ADC_MAX_VOLTAGE 4.1
  #define U_DEVIDER 4.1 //5.05
#endif

#include "FS.h"
#include <LittleFS.h>

AsyncWebServer server(80);

#include <Wire.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bmp;

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);
#define MAX1704x

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <time.h>
time_t now;

// Define NTP Client to get time
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

char WEATHERNODE[20];
int nodeNumber = 0;

// File paths to save input values permanently
const char* ssidPath  = "/ssid.txt";
const char* passPath  = "/pass.txt";
const char* mqttPath  = "/mqtt.txt";
const char* idPath    = "/id.txt";
const char* adcPath   = "/adc.txt";
const char* sleepPath = "/sleep.txt";
const char* tempPath  = "/temp.txt";

// Replace with your network details
//Variables to save values from HTML form
String ssid = "";
String pass;
String ip;
String mqtt_server = "hap-nodejs";
float_t adcCorrection = 2.6;
float_t tempCorrection = -1.5;
const char* mqtt_topic = WEATHERNODE;
int nodeId = 0;
int sleepDuration = DEEPSLEEPDURATION;

long lastMsg = 0;
int value = 0;
char json[512];

WiFiClient espClient;
PubSubClient client(espClient);

// Temporary variables
static char celsiusTemp[7];
static char heatIndexTemp[7];
static char humidityTemp[7];
static char pressureTemp[7];
static char dewPTemp[7];
static char voltageTemp[7];

float dst,bt,bp,ba;
char dstmp[20],btmp[20],bprs[20],balt[20];
int clientCounter;
int publishCounter;

void i2cScanner() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Serial.print(".");
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      nDevices++;
      Serial.printf("\ndevice found at 0x%02x\n", address);
    }
    else if (error==4) {
      Serial.printf("\nUnknown error at address 0x%02x\n", address);
    }
  }
  if (nDevices == 0) {
    Serial.println("\nNo I2C devices found\n");
  }
  else {
    Serial.println("\ndone\n");
  }
}

float convertCtoF(float c) {
  return c * 1.8 + 32;
}

float convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

float calcDewpoint(float humi, float temp) {
  //Serial.println("calcDewPoint()");

  float k;
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
}

void reconnectMqtt() {
  // Loop until we're reconnected
  Serial.println("reconnectMqtt()");
  while (!client.connected() && clientCounter < 5) {
    Serial.printf("Attempting MQTT connection to %s\n", mqtt_server.c_str());
    // Attempt to connect
    if (client.connect("ESPClient")) {
      Serial.println("connected");
    } else {
      Serial.printf("failed, rc=%d\n", client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 3 seconds before retrying
      delay(5000);
      clientCounter++;
    }
  }
  if( !client.connected() && clientCounter == 5 ) {
    Serial.println("could not connect to MQTT server, restart ESP");
    ESP.restart();
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getVoltage() {
  Serial.println("getVoltage()");
  float busvoltage;
  #ifndef MAX1704x
    int rawVoltage = analogRead(ADC_PIN);
    Serial.printf("rawVoltage:     %d\n", rawVoltage);
    #ifdef DEBUG
    Serial.printf("ADC_MAX_VOLTAGE: %f\n", ADC_MAX_VOLTAGE);
    Serial.printf("ADC_RESOLUTION:  %f\n", ADC_RESOLUTION);
    Serial.printf("U_DEVIDER:       %f\n", U_DEVIDER);
    Serial.printf("adcCorrection:   %f\n", adcCorrection);
    Serial.printf("without corr.:   %1.1f\n", ((((float)rawVoltage * ADC_MAX_VOLTAGE) / ADC_RESOLUTION) * U_DEVIDER) );
    #endif
    busvoltage = ((((float)rawVoltage * ADC_MAX_VOLTAGE) / ADC_RESOLUTION) * U_DEVIDER) + adcCorrection;
    if( rawVoltage == 1024 ) {
      Serial.println("rawVoltage == 1024 -> seems to be USB voltage of 5V");
      busvoltage = 5.0;
    }
  #else
    busvoltage = fuelGauge.getVoltage();
  #endif
  Serial.printf("busvoltage: %1.2f\n", busvoltage);
  do_publish = true;
  return busvoltage;
}

bool getSensorData() {
  Serial.println("getSensorData()");
  if( bmp_init ) {
    float vin = getVoltage();
    snprintf (voltageTemp, 6, "%1.2f", vin);
    Serial.printf("Vin: %f\n", vin);

    Serial.println("before takeForcedMeasurement");
    Serial.printf("bmp.takeForcedMeasurement() = %s\n", bmp.takeForcedMeasurement() ? "TRUE" : "FALSE");
    float bp =  bmp.readPressure()/100;
    Serial.printf("readPressure: %fhPa\n", bp);
    float h =   bmp.readHumidity();
    Serial.printf("readHumidity: %f%%\n", h);
    float t =   bmp.readTemperature(); 
    Serial.printf("readTemperature: %fC\n", t);
    t = t + tempCorrection;
    Serial.printf("readTemperature after tempCorrection (%f): %fC\n", tempCorrection, t);
    float hic = computeHeatIndex(t, h, false);       
    Serial.printf("heatIndex: %f\n", hic);
    Serial.printf("dew point: %fC\n", calcDewpoint(h, t));
    snprintf (celsiusTemp, 6, "%2.1f", t);             
    snprintf (heatIndexTemp, 6, "%2.1f", hic);
    snprintf (humidityTemp, 6, "%3.0f", h);
    snprintf (pressureTemp, 6, "%4.0f", bp);
    snprintf (dewPTemp, 6, "%2.1f", calcDewpoint(h, t));
  }
  return true;
}

bool getJSON() {
  Serial.println("getJSON()");
  StaticJsonDocument<500> doc;

  bool status = getSensorData();

  Serial.print("now: ");
  time(&now);
  Serial.println( now );
  
  Serial.println("create JSON data");

  Serial.printf("clientCounter: %d\n", clientCounter);
  Serial.printf("publishCounter: %d\n", publishCounter);

  doc["time"] = now;
  doc["temperature"] = celsiusTemp;
  doc["heatindex"] = heatIndexTemp;
  doc["dewpoint"] = dewPTemp;
  doc["humidity"] = humidityTemp;
  doc["pressure"] = pressureTemp;
  doc["voltage"] = voltageTemp;
  doc["clientcounter"] = clientCounter;
  doc["publishcounter"] = publishCounter;
  serializeJson(doc, json);

  Serial.print( "getJSON() status: " );
  Serial.println( status );
  return status;
}

// Delete file from FS
bool deleteFile(const char * path) {
  Serial.printf("deleting file: %s\n", path);
  if( LittleFS.exists(path) )
    return LittleFS.remove(path);
  else
    Serial.println("file does not exist");
  return false;
}

// Read File from FS
String readFile(const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = LittleFS.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }

  String fileContent;
  while(file.available()){
    fileContent = file.readString(); // readStringUntil('\n');
    break;     
  }
  file.close();
  return fileContent;
}

// Write file to LittleFS
void writeFile(const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = LittleFS.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    //Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
  file.close();
}

bool getWiFiManager() {
  if( digitalRead(MANAGER_PIN) == 0 ) {
    return true;
  }
  return false;
}

bool getReset() {
  if( digitalRead(RESET_PIN) == 0 ) {
    return true;
  }
  return false;
}

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "mqttserver";
const char* PARAM_INPUT_4 = "nodeid";
const char* PARAM_INPUT_5 = "adccorrection";
const char* PARAM_INPUT_6 = "sleepduration";
const char* PARAM_INPUT_7 = "tempcorrection";

String webServerProcessor(const String& var) {
  Serial.printf("webServerProcessor(\"%s\")\n", var.c_str());
  if (var == "NODE_ID"){
    //int nodeNumber = getNodeId();
    Serial.printf("NODE_ID: %d\n", nodeId);
    return String(nodeId);
  }
  if (var == "MQTTSERVER"){
    Serial.printf("MQTTSERVER: %s\n", mqtt_server.c_str());
    return mqtt_server;
  }
  if (var == "SSID"){
    Serial.printf("SSID: %s\n", ssid.c_str());
    return ssid.c_str();
  }
  if (var == "PASS"){
    Serial.printf("PASS: %s\n", pass.c_str());
    return pass.c_str();
  }
  if (var == "NODEID"){
    Serial.printf("NODEID: %d\n", nodeId);
    return String(nodeId);
  }
  if (var == "ADCCORRECTION"){
    Serial.printf("ADCCORRECTION: %f\n", adcCorrection);
    return String(adcCorrection);
  }
  if (var == "TEMPCORRECTION"){
    Serial.printf("TEMPCORRECTION: %f\n", tempCorrection);
    return String(tempCorrection);
  }
  if (var == "SLEEPDURATION"){
    Serial.printf("SLEEPDURATION: %d\n", sleepDuration);
    return String(sleepDuration);
  }
  return String();
}

void startWiFiManager() {
  // Connect to Wi-Fi network with SSID and password
  Serial.println("Setting AP (Access Point)");
  // NULL sets an open Access Point
  WiFi.softAP("WeatherNode-WiFi-Manager", NULL);
  Serial.printf("connect to: %s\n", "WeatherNode-WiFi-Manager");

  digitalWrite(LED_BUILTIN, LOW);

  IPAddress IP = WiFi.softAPIP();
  WiFi.setHostname("WeatherNode");
  Serial.print("AP IP address: ");
  Serial.println(IP); 

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("server.on(\"/\", HTTP_GET, ...)");
    request->send(LittleFS, "/wifimanager.html", "text/html", false, webServerProcessor);
  });

  server.serveStatic("/", LittleFS, "/");

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    Serial.println("server.on(\"/\", HTTP_POST, ...)");
    int params = request->params();
    for(int i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()){
        // HTTP POST ssid value
        if (p->name() == PARAM_INPUT_1) {
          ssid = p->value().c_str();
          Serial.printf("SSID set to: %s\n", ssid.c_str());
          // Write file to save value
          writeFile(ssidPath, ssid.c_str());
        }
        // HTTP POST pass value
        if (p->name() == PARAM_INPUT_2) {
          pass = p->value().c_str();
          Serial.printf("Password set to: %s\n", pass.c_str());
          // Write file to save value
          writeFile(passPath, pass.c_str());
        }
        // HTTP POST mqttserver value
        if (p->name() == PARAM_INPUT_3) {
          mqtt_server = p->value().c_str();
          Serial.printf("MQTT Server set to: %s\n", mqtt_server.c_str());
          // Write file to save value
          writeFile(mqttPath, mqtt_server.c_str());
        }
        // HTTP POST idBase value
        if (p->name() == PARAM_INPUT_4) {
          nodeId = p->value().toInt();
          Serial.printf("nodeId set to: %d\n", nodeId);
          // Write file to save value
          writeFile(idPath, String(nodeId).c_str());
        }
        // HTTP POST adcCorrection value
        if (p->name() == PARAM_INPUT_5) {
          adcCorrection = p->value().toFloat();
          Serial.printf("adcCorrection set to: %f\n", adcCorrection);
          // Write file to save value
          writeFile(adcPath, String(adcCorrection).c_str());
        }
        // HTTP POST sleepduration value
        if (p->name() == PARAM_INPUT_6) {
          sleepDuration = p->value().toInt();
          Serial.printf("sleepDuration set to: %d\n", sleepDuration);
          // Write file to save value
          writeFile(sleepPath, String(sleepDuration).c_str());
        }
        // HTTP POST sleepduration value
        if (p->name() == PARAM_INPUT_7) {
          tempCorrection = p->value().toFloat();
          Serial.printf("tempCorrection set to: %f\n", tempCorrection);
          // Write file to save value
          writeFile(tempPath, String(tempCorrection).c_str());
        }
      }
    }
    request->send(200, "text/plain", "Done. ESP will restart");
    delay(3000);
    ESP.restart();
  });
  server.begin();
  Serial.println("start WiFiManager finished");
}

bool initWiFi() {
    if(ssid == "" || pass == ""){
        Serial.println("Undefined SSID or password");
        return false;
    }
    if( getWiFiManager() ){
        Serial.println("start WifiManager via PIN");
        return false;
    }
    // if( getReset() ){
    //     Serial.println("start WifiManager via Reset PIN");
    //     deleteFile(ssidPath);
    //     deleteFile(passPath);
    //     writeFile(mqttPath, "hap-nodejs");
    //     writeFile(idPath, "0");
    //     writeFile(adcPath, "0.0");
    //     writeFile(sleepPath, "900000000");
    //     ESP.restart();
    //     return false;
    // }
    return true;
}

// only runs once on boot
void setup() {
  int myLoopCount = 0;
  clientCounter = publishCounter = 0;

  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  delay(10);
  Serial.println("");

  #ifdef ESP32
    Serial.println();
    Serial.printf("Chip Model : %s\n", ESP.getChipModel());
    Serial.printf("Chip rev.  : %d\n", ESP.getChipRevision());
    Serial.printf("CPU Freq.  : %d\n", ESP.getCpuFreqMHz());
    Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason){
      case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
      case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
      case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
      case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
      default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }

  #else
    Serial.println();
    Serial.printf("Chip ID     : %d\n", ESP.getChipId());
    Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
    Serial.printf("CPU Freq.   : %d\n", ESP.getCpuFreqMHz());
    Serial.printf("Boot Version: %d\n", ESP.getBootVersion());
    rst_info *rinfo = ESP.getResetInfoPtr();
    Serial.printf("ResetInfo.reason = %d: %s\n\n", (*rinfo).reason, ESP.getResetReason().c_str());
  #endif

  Serial.println("LittleFS.begin()");
  if (!LittleFS.begin()) {
    Serial.println(F("An error occurred when attempting to mount the flash file system"));
  } else {
    Serial.println("FS contents:");

    File root = LittleFS.open("/", "r");
    File file = root.openNextFile();
    while(file)
    {  
      String fileName = file.name();
      size_t fileSize = file.size();
      Serial.printf("FS File: %s, size: %d\n", fileName.c_str(), fileSize);
      file = root.openNextFile();
    }
    Serial.printf("\n");
  }
  Serial.println("LittleFS init finished");

  // read existing files
  ssid = readFile(ssidPath);
  Serial.println(ssid);
  pass = readFile(passPath);
  Serial.println(pass);
  mqtt_server = readFile(mqttPath);
  Serial.println(mqtt_server);
  nodeId = readFile(idPath).toInt();
  Serial.println(nodeId);
  adcCorrection = readFile(adcPath).toFloat();
  Serial.println(adcCorrection);
  tempCorrection = readFile(tempPath).toFloat();
  Serial.println(tempCorrection);
  sleepDuration = readFile(sleepPath).toInt();
  Serial.println(sleepDuration);
  //getVoltage();

  sprintf(WEATHERNODE, "WeatherNode%d", nodeId);
  Serial.println(WEATHERNODE);

  Wire.begin();
  i2cScanner();
  
  Serial.println("init MAX1704x");
  max_init = fuelGauge.begin();
  if( !max_init ) {
    Serial.println("error init MAX1704x");
  }

  Serial.println("init BME280");

  bmp_init = bmp.begin(BME280_ADDRESS_ALTERNATE);
  Serial.printf("bme280(BME280_ADDRESS_ALTERNATE) init: %s\n", bmp_init ? "TRUE" : "FALSE");
  if( !bmp_init ) {
    do_publish = false;
    Adafruit_I2CDevice *i2c_dev = new Adafruit_I2CDevice(BME280_ADDRESS_ALTERNATE, &Wire);
    if (!i2c_dev->begin())
      Serial.println("i2c_dev begin() faulty");
    else
      Serial.println("i2c_dev begin ok");
    uint8_t buffer[1];
    buffer[0] = uint8_t(BME280_REGISTER_CHIPID);
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
    Serial.printf("BME280_REGISTER_CHIPID: 0x%02X should be 0x60\n", buffer[0]);
  }
  if( bmp_init ) {
    Serial.println("config BME280");
    bmp.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF,
                    Adafruit_BME280::STANDBY_MS_1000);  

    bmp.takeForcedMeasurement();
    Serial.printf("readPressure: %fhPa\n", bmp.readPressure()/100);
    Serial.printf("readHumidity: %f%%\n", bmp.readHumidity());
    Serial.printf("readTemperature: %fC\n", bmp.readTemperature());
    Serial.printf("readTemperature after correction(%f): %fC\n", tempCorrection, bmp.readTemperature()+tempCorrection);
  }

  pinMode(MANAGER_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  Serial.printf("MANAGER_PIN: %d\n", digitalRead(MANAGER_PIN));
  Serial.printf("RESET_PIN:   %d\n", digitalRead(RESET_PIN));
  
  Serial.printf("Voltage = %f\n", fuelGauge.getVoltage());
  Serial.printf("Battery Percentage %f\n", fuelGauge.getSOC());

  // Connecting to WiFi network
  if( initWiFi() ) {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  
    //WiFi.forceSleepWake();
    delay(1);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.hostname(mqtt_topic);
    WiFi.begin(ssid.c_str(), pass.c_str());
    
    while (WiFi.status() != WL_CONNECTED and myLoopCount < 20) {
        delay(500);
        Serial.print(".");
        myLoopCount++;
    }

    if( WiFi.status() != WL_CONNECTED ) {
      Serial.println("could not connect to WiFi");
      //Serial.println("deleting WiFi config files");
      //deleteFile(ssidPath);
      //deleteFile(passPath);
      Serial.println("restarting ESP");
      ESP.restart();
    }
    Serial.println("");
    Serial.println("WiFi connected");

    // Printing the ESP IP address
    Serial.println(WiFi.localIP());

    //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    //char buf[50];
    //mqtt_server.toCharArray(buf, 50);
    client.setServer(mqtt_server.c_str(), 1883);
  }
  else {
    //Serial.println("deleting WiFi config files");
    //deleteFile(ssidPath);
    //deleteFile(passPath);
    startWiFiManager();
  }
  // wifi_set_sleep_type(LIGHT_SLEEP_T);

}

int rawVoltageLastValue = 0;

// runs over and over again
void loop() {
  if( WiFi.getMode() ==  WIFI_STA ) {
    if (!client.connected()) {
        reconnectMqtt();
    }
    client.loop();

    time(&now);
    tm tm;
    localtime_r(&now, &tm);
    char res[32];
    sprintf(res, "%02d.%02d.%04d %02d:%02d:%02d", tm.tm_mday, tm.tm_mon, tm.tm_year, tm.tm_hour, tm.tm_min, tm.tm_sec);

    Serial.println( now );
    Serial.println( res );
    
    bool status = getJSON();
    Serial.printf("Publish to topic: %s\n", mqtt_topic);
    Serial.printf("Publish message: %s\n", json);
    Serial.println(sizeof(json));
    if( status == true ) {
      if( do_publish ) {
        if ( client.publish(mqtt_topic, json, false) == true ) {
          Serial.println("succcess");
          delay(1000);
          client.disconnect();
        #ifdef DO_DEEPSLEEP    
          Serial.printf("go to deep sleep for %ds\n", sleepDuration/(1000*1000));
          #ifdef ESP32
            esp_sleep_enable_timer_wakeup(DEEPSLEEPDURATION);
            esp_deep_sleep_start();
          #else
            ESP.deepSleep(sleepDuration, WAKE_RF_DEFAULT);
          #endif
        #else
          Serial.println("delay for 1000ms");
          delay(1000);
        #endif
        } else {
          Serial.println("publish failed");
          delay(1000);
          publishCounter++;
          ESP.restart();
        }
        Serial.print("do publish to: ");
        Serial.println(WEATHERNODE);
        Serial.println(json);
        client.disconnect();
      /*
      #ifdef DO_DEEPSLEEP    
        Serial.print("go to deep sleep for "); 
        Serial.print(DEEPSLEEPDURATION/(1000*1000));
        Serial.println("s");
        //esp_sleep_enable_timer_wakeup( DEEPSLEEPDURATION );
        //esp_deep_sleep_start();
        //ESP.deepSleep(DEEPSLEEPDURATION, WAKE_RF_DEFAULT);
        #ifdef ESP32
          esp_sleep_enable_timer_wakeup(DEEPSLEEPDURATION);
          esp_deep_sleep_start();
        #else
          ESP.deepSleep(DEEPSLEEPDURATION, WAKE_RF_DEFAULT);
        #endif
      #else
        Serial.println("delay for 1000ms");
        delay(1000);
      #endif
      */
      }
      else {
        Serial.println("publish not done due to failed HW init");
        Serial.printf("go to deep sleep for rebooting %ds\n", DEEPSLEEPDURATION/(1000*1000)); 
        #ifdef ESP32
          esp_sleep_enable_timer_wakeup(DEEPSLEEPDURATION);
          esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
          esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
          esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
          esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
          esp_deep_sleep_start();
        #else
          #ifdef MAX1704x
            fuelGauge.sleep();
          #endif
          ESP.deepSleep(DEEPSLEEPDURATION, WAKE_RF_DEFAULT);
        #endif
      }
    } else {
        Serial.println("status != true");
        delay(1000);
    }
  }
  else {
    #ifdef DEBUG
      #ifndef MAX1704x
        int rawVoltage = publishCounter = analogRead(ADC_PIN);
        if( rawVoltageLastValue != rawVoltage ) {
          Serial.printf("rawVoltage:     %d\n", rawVoltage);
          rawVoltageLastValue = rawVoltage;
        }
        Serial.printf("without corr.: %1.1f busvoltage: %1.1f\n", ((((float)rawVoltage * ADC_MAX_VOLTAGE) / ADC_RESOLUTION) * U_DEVIDER), ((((float)rawVoltage * ADC_MAX_VOLTAGE) / ADC_RESOLUTION) * U_DEVIDER) + adcCorrection);
      #else
        Serial.printf("Voltage = %f\n", fuelGauge.getVoltage());
        Serial.printf("Battery Percentage %f\n", fuelGauge.getSOC());
      #endif
    #endif
  }
}   
