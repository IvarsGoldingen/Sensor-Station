#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <CO2_sens.h>
#include <FirebaseESP32.h>
#include <EEPROM.h>
#include <Adafruit_BME280.h>
//#include <WiFi.h>




//debug serial print 0 = no 1 yes
#define DEBUG 1



/***********EEPROM defines*****************/
//EEPROM size
#define EEPROM_BYTES 10
//Addresses for baseline values
#define EEPADD_BASE_EXISTS 1
#define EEPADD_CO2_1 2
#define EEPADD_CO2_2 3
#define EEPADD_TVOC_1 4
#define EEPADD_TVOC_2 5

//FIREBASE and WIFI
#define FIREBASE_HOST "https://esp32test-ef408.firebaseio.com/"
#define FIREBASE_AUTH "AWfs3pD2ZeIbZ1tw8hNWGZ7VXgY8GyGrExGPXKIF"
#define MAX_WIFI_CONNECT_TIME_S 10
#define MAX_WIFI_CONNECT_TIME_MS MAX_WIFI_CONNECT_TIME_S*1000
#define WRITE_TO_FB_MS 1000

/***********DHT defines*****************/
// Digital pin connected to the DHT sensor
#define DHTPIN 33     
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//How often to read from DHT sensor
#define READ_DHT_S 10
#define READ_DHT_MS READ_DHT_S*1000

/***********SGP defines*****************/
#define I2C_SDA 26
#define I2C_SCL 25
//How often to read from SGP sensor
#define READ_SGP_S 1
#define READ_SGP_MS READ_SGP_S*1000
//How often to write humidity to SGP sensor
#define WRITE_HUM_SGP_S 60
#define WRITE_HUM_MS WRITE_HUM_SGP_S*1000
#define DEFAULT_CO2_BASE 35694
#define DEFAULT_TVOC_BASE 37064
#define STORE_BASE_VALUES_M 60
#define STORE_BASE_VALUES_MS STORE_BASE_VALUES_M*60*1000

/***********NTP time*****************/
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_S = 7200;
const int   DAYLIGHT_OFFSET_S = 3600;
struct tm timeinfo;

/***********wifi and Firebase values*****************/
const char* ssid = "HUAWEI-B315-3EDC";
const char* password =  "A3E10YT5MA2";
const unsigned long LOG_FB_INTERVAL_S = 600;
const unsigned long LOG_FB_INTERVAL_MS = LOG_FB_INTERVAL_S * 1000;
FirebaseData firebaseData;
FirebaseJson json;
FirebaseJson json2;
bool wifiConnected = false;
unsigned long lastWriteToFB = 0;
unsigned long lastFbLog = 0;
unsigned long logNr = 0;


//I2C object
TwoWire I2C = TwoWire(0);

/***********LOG values*****************/
unsigned long lastLog = 0;
unsigned const long LOG_PERIOD_S = 300;
unsigned const long LOG_PERIOD_MS = LOG_PERIOD_S * 60;

/***********SGP values*****************/
//SGP sensor object
CO2_sens CO2s;
//Values from SGP sensor are stored here
uint16_t arrCO2_TVOC [2] = {0,0};
//last time SGP was read
unsigned long lastSGPReadMs = 0;
//last time humidity was set in SGP
unsigned long lastSGPhumSet = 0;
//Was last measurement valid
bool lastSGPReadValid = false;
//last time base values were read
unsigned long lastSGPBaseRead = 0;


/***********DHT values*****************/
//DHT sensor object
DHT dht(DHTPIN, DHTTYPE);
//Values from DHT sensor
double humidity = -100.0;
double temperature = -100.0;
//Indicates whether the last read had correct values
bool tempHumValid = false;
//last time DHT was read
unsigned long lastDHTReadMs = 0;

bool test = true;

void logFb();
void clearBaseValues();
void storeBaseValues();
void initCO2baseValues();
void debugPrintln(String text);
void debugPrint(String text);
void updateFbRealTimeData();
void setupFireBase();
void connectWifi();
void measureAirQuality();
void setHumidityInCO2Sens();
void measureHumTemp();
void readSerial();
uint32_t getAbsoluteHumidity(double temperature, double humidity);
bool timePassed(unsigned long* prevTime, int intervalMs);

void setup() {
  Serial.begin(115200);
  EEPROM.begin(10);
  connectWifi();
  configTime(GMT_OFFSET_S, DAYLIGHT_OFFSET_S, NTP_SERVER);
  setupFireBase();
  //I2C for SGP sensor
  I2C.begin(I2C_SDA, I2C_SCL);
  //SGP sensor
  CO2s.setup(&I2C);
  //init CO2 sensor with base values stored in flash
  initCO2baseValues();
  //Humidity and temp sensor setup
  dht.begin();

  logFb();
  while(1);
}

void loop() {
  measureAirQuality();
  measureHumTemp();
  setHumidityInCO2Sens();
  updateFbRealTimeData();
  storeBaseValues();
  readSerial();
  logFb();
}

//delete base values from flash
void clearBaseValues(){
  for (int i = 0; i < EEPROM_BYTES; i++){
    EEPROM.write(i,0xff);
  }
  EEPROM.commit();
}

//Read serial for incomming commands
void readSerial(){
  while (Serial.available()){
    //read data from serial while available
    char cmd = Serial.read();
    switch (cmd)
    {
    case 'd':
      debugPrintln("Clearing flash");
      clearBaseValues();
      break;
    case 'r':
      debugPrintln("Recalibration after next restart");
      EEPROM.write(EEPADD_BASE_EXISTS, 2);
      EEPROM.commit();
      break;
    default:
      debugPrintln("Unknown char cmd");
      break;
    }
  }
}

//Set base values in the SGP sensor
void initCO2baseValues(){
  bool setbaseValues = false;
  uint16_t CO2Base = 0;
  uint16_t TVOCBase = 0;
  //check if there is a value stored in EEPROM
  byte calibFunc = EEPROM.read(EEPADD_BASE_EXISTS);
  if (calibFunc == 1){
    debugPrintln("Reading base values stored in EEPROM");
    byte CO2byte1 = EEPROM.read(EEPADD_CO2_1);
    byte CO2byte2 = EEPROM.read(EEPADD_CO2_2);
    byte TVOCbyte1 = EEPROM.read(EEPADD_TVOC_1);
    byte TVOCbyte2 = EEPROM.read(EEPADD_TVOC_2);
    CO2Base = (int)(CO2byte1 << 8) + CO2byte2;
    TVOCBase = (int)(TVOCbyte1 << 8) + TVOCbyte2;
    debugPrint("CO2 Base set:");
    debugPrintln((String)CO2Base);
    debugPrint("TVOC Base set:");
    debugPrintln((String)TVOCBase);
    setbaseValues = true;
  } else if(calibFunc == 2){
    debugPrintln("Skipping initial values");
  } else {
    //use hard coded base values
    CO2Base = DEFAULT_CO2_BASE;
    TVOCBase = DEFAULT_TVOC_BASE;
    debugPrintln("No base value stored in EEPROM");
    debugPrint("CO2 Base set:");
    debugPrintln((String)DEFAULT_CO2_BASE);
    debugPrint("TVOC Base set:");
    debugPrintln((String)DEFAULT_TVOC_BASE);
    setbaseValues = true;
  }
  if (setbaseValues){
    if (CO2s.setBaseLine(CO2Base, TVOCBase)){
      debugPrintln("Base set succesfully");
    } else {
      debugPrintln("Base set failed");
    }
  }
}

//Store the autocalibration values from SGP sensor
//recommended to do once an hour
void storeBaseValues(){
  if (timePassed(&lastSGPBaseRead, STORE_BASE_VALUES_MS)){
    debugPrintln("Reading base values");
    uint16_t  bLine[2];
    CO2s.getBaseline(bLine);
    debugPrint("eCO2 base:"); debugPrintln((String)bLine[0]);
    debugPrint("TVOC base:"); debugPrintln((String)bLine[1]);
    byte CO2byte1 = bLine[0] >> 8;
    byte CO2byte2 = bLine[0];
    byte TVOCbyte1 = bLine[1] >> 8;
    byte TVOCbyte2 = bLine[1];
    EEPROM.write(EEPADD_BASE_EXISTS, 1);
    EEPROM.write(EEPADD_CO2_1, CO2byte1);
    EEPROM.write(EEPADD_CO2_2, CO2byte2);
    EEPROM.write(EEPADD_TVOC_1, TVOCbyte1);
    EEPROM.write(EEPADD_TVOC_2, TVOCbyte2);
    EEPROM.commit();
    debugPrintln("Base values saved");
  }
}

//Makes the sensor more accurate
void setHumidityInCO2Sens(){
  //Write only after a certain time
  if (timePassed(&lastSGPhumSet, WRITE_HUM_MS)){
    if (tempHumValid){
      if (!CO2s.setHumidity(getAbsoluteHumidity(temperature, humidity))){
        Serial.println("Failed to set humidity");
      } else {
        Serial.println("Humidity set");
      }
    }
  }
}

//Read from DHT sensor
void measureHumTemp(){
  //Write only after a certain time
  if (timePassed(&lastDHTReadMs, READ_DHT_MS)){
    //Read humidity
    humidity = dht.readHumidity();
    //Read temperature as Celsius (the default)
    temperature = dht.readTemperature();

    // Check if any reads failed
    if (isnan(humidity) || isnan(temperature) || humidity > 100.0) {
      Serial.println(F("Failed to read from DHT sensor!"));
      tempHumValid = false;
    } else {
      Serial.print("T: "); Serial.print(temperature); Serial.println(" C");
      Serial.print("H: "); Serial.print(humidity); Serial.println(" %");
      tempHumValid = true;
    }
  }
}

//Read from SGP sensor
void measureAirQuality(){
  if (timePassed(&lastSGPReadMs, READ_SGP_MS)){
    bool success = CO2s.measure(arrCO2_TVOC);
    if (success){
      lastSGPReadValid = true;
      Serial.print("TVOC "); Serial.print(arrCO2_TVOC[1]); Serial.print(" ppb\t");
      Serial.print("eCO2 "); Serial.print(arrCO2_TVOC[0]); Serial.println(" ppm");
    } else {
      lastSGPReadValid = false;
      Serial.println("SGP measurement failed");
    }
  }
}

//This value is passed to the SGP sensor
/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(double temperature, double humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}
bool timePassed(unsigned long* prevTime, int intervalMs){
  unsigned long currentTime = millis();
  unsigned long timePassed = currentTime - *prevTime;
  if (timePassed >= intervalMs){
    *prevTime = currentTime;
    return true;
  }
  return false;
}

//Connect to WiFi
void connectWifi(){
  unsigned long connectWifiStartTime = millis();
  unsigned long writeCharTime = connectWifiStartTime;
  bool connectTimeOut = false;
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (!connectTimeOut && !wifiConnected){
    if ((millis() - writeCharTime) > 25){
      //connecting indicator
      Serial.print(".");
      writeCharTime = millis();
    }
    if (WiFi.status() == WL_CONNECTED){
      //Connection successful
      wifiConnected = true;
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP()); 
    }
    if (millis() - connectWifiStartTime > MAX_WIFI_CONNECT_TIME_MS){
      //timed out of wifi connection
      if (!wifiConnected){
        connectTimeOut = true;
        Serial.println("");
        Serial.println("WiFi connect time out");
      }
    }
  }
}

void setupFireBase(){
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}

//sava data with timestamps in FB
void logFb() {
  if (timePassed(&lastFbLog, LOG_FB_INTERVAL_MS)){
    if (tempHumValid){
      json2.set("/temperature", temperature);
      json2.set("/humidity", humidity);
    } else {
      json2.set("/temperature", -1);
      json2.set("/humidity", -1);
    }
    if (lastSGPReadValid){
      json2.set("/TVOC", arrCO2_TVOC[1]);
      json2.set("/CO2", arrCO2_TVOC[0]);
    }
    json2.set("Time2", "timestamp");
    Firebase.updateNode(firebaseData, "/Log2/" + (String)logNr,json2);
    //Firebase.setTimestamp(firebaseData, "/Log/" + String(logNr)+ "/Time");
    logNr++;
  }
  
}

//Update real time data in FB
void updateFbRealTimeData(){  
  if (timePassed(&lastWriteToFB, WRITE_TO_FB_MS)){
    json.set("/DHT_valid", tempHumValid);
    json.set("/SGP_valid", lastSGPReadValid);

    if (tempHumValid){
      json.set("/temperature", temperature);
      json.set("/humidity", humidity);
    }
    if (lastSGPReadValid){
      json.set("/TVOC", arrCO2_TVOC[1]);
      json.set("/CO2", arrCO2_TVOC[0]);
    }
    Firebase.updateNode(firebaseData,"/SensorStation",json);    
  }
}

//Print ln if debug mode enabled
void debugPrintln(String text){
  if (DEBUG == 1){
    Serial.println(text);
  }
}

//Print if debug mode enabled
void debugPrint(String text){
  if (DEBUG == 1){
    Serial.print(text);
  }
}