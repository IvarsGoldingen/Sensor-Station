#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <CO2_sens.h>
#include <FirebaseESP32.h>
#include <EEPROM.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal.h>
#include <Secrets.h>
//debug serial print 0 = no 1 yes
#define DEBUG 0

//TODO:check wifi and if there is not auto connect

/***********EEPROM defines*****************/
//EEPROM size
#define EEPROM_BYTES 10
//Addresses for baseline values
#define EEPADD_BASE_EXISTS 1
#define EEPADD_CO2_1 2
#define EEPADD_CO2_2 3
#define EEPADD_TVOC_1 4
#define EEPADD_TVOC_2 5

//FIREBASE
#define MAX_WIFI_CONNECT_TIME_S 5
#define MAX_WIFI_CONNECT_TIME_MS MAX_WIFI_CONNECT_TIME_S*1000
#define WRITE_TO_FB_MS 1000

/***********DHT defines - temp and humidity*****************/
// Digital pin connected to the DHT sensor
#define DHTPIN 33     
// Uncomment whatever type you're using!
#define DHTTYPE DHT22   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//How often to read from DHT sensor
#define READ_DHT_S 10
#define READ_DHT_MS READ_DHT_S*1000

/***********SGP defines - CO2 and TVOC*****************/
//How often to read from SGP sensor
#define READ_SGP_S 1
#define READ_SGP_MS READ_SGP_S*1000
//How often to write humidity to SGP sensor
#define WRITE_HUM_SGP_S 3600
#define WRITE_HUM_MS WRITE_HUM_SGP_S*1000
#define DEFAULT_CO2_BASE 35694
#define DEFAULT_TVOC_BASE 37064
//NUMBER OF TIMES BASE VALUE CALL MAST BE CALLED FOR THE FIRST STORE TO ACTUALLY HAPPEN
#define STORE_BASE_VALUES_START 72
#define STORE_BASE_VALUES_M 60
#define STORE_BASE_VALUES_MS STORE_BASE_VALUES_M*60*1000

/***********BMP constants - pressure and temp*****************/
const long READ_F_BMP_P_T_S = 10;
const long READ_F_BMP_P_T_MS = READ_F_BMP_P_T_S * 1000;

/***********NTP time*****************/
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_S = 7200;
const int   DAYLIGHT_OFFSET_S = 3600;
struct tm timeinfo;

/***********wifi and Firebase values*****************/
const unsigned long LOG_FB_INTERVAL_S = 600;
const unsigned long LOG_FB_INTERVAL_MS = LOG_FB_INTERVAL_S * 1000;
FirebaseData firebaseData;
FirebaseJson json;
FirebaseJson json2;
bool wifiConnected = false;
unsigned long lastWriteToFB = 0;
unsigned long lastFbLog = 0;
unsigned long logNr = 0;

//I2C
#define I2C_SDA 26
#define I2C_SCL 25
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
//Counter that delays the base value store. Needed according to the datasheet,
unsigned int startBaseValueStoreCntr = 0;


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

/***********BMP280 values - pressure sensor*****************/
Adafruit_BMP280 bmp = Adafruit_BMP280(&I2C);
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
unsigned long lastPressureRead = 0;
double temperature2 = -100.0;
double pressure = -100.0;
bool bmpReadValid = false;

/***********Display values*****************/
const int RS = 15, EN = 2, D4 = 0, D5 = 4, D6 = 16, D7 = 17;
const byte MAX_SCREEN_NUMBER = 3;
const byte PIN_BACKLIGHT = 19;
unsigned const long LCD_UPDATE_FRQ_MS = 1000;
unsigned const long LCD_SLEEP_DELAY_MS = 30000;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long lastLCDupdate = 0;
byte screenNumber = 1;
bool lcdOn = false;

/***********Button values*****************/
const byte BUTTON_PIN = 5;
//Delay for button to be deactivated. Used for debounce on button realease
const unsigned long BTN_DEACTIVATION_TIME_MS = 100;
const unsigned long SHORT_PRESS_TIME = 40;
const unsigned long LONG_PRESS_TIME = 1400;
const byte SHORT_PRESS = 1;
const byte LONG_PRESS = 2;
unsigned long lastButtonPress = 0;
unsigned long btnTimeOfLastDeactivation = 0;
bool btnDeactivated = false;
volatile bool btnWasPressed = false;
volatile unsigned long timeOfPress = 0;


void setupInterruptBtn();
void checkBtn();
void IRAM_ATTR buttonISR();
void lcdCheckSleep();
void lcdTurnOn();
void lcdTurnOff();
void buttonPress(byte);
void updateLCD();
void measurePressureTemp();
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
void lcdWifiAnim (int step);

//Does not work well since a lot is going on in paralell
//changed to work from interrupt
//ButtonIB btn(BUTTON_PIN, *buttonPress);

void setup() {
  pinMode(PIN_BACKLIGHT, OUTPUT);
  //Initiate LCD screen
  lcdTurnOn();
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
  //Init pressure and temperature sensor
  /* Default settings from datasheet. */
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();
  setupInterruptBtn();
}


void setupInterruptBtn(){
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //ISR for button
  attachInterrupt(BUTTON_PIN, buttonISR, RISING);
}

void loop() {
  measureAirQuality();
  measureHumTemp();
  setHumidityInCO2Sens();
  updateFbRealTimeData();
  storeBaseValues();
  readSerial();
  measurePressureTemp();
  logFb();
  updateLCD();
  lcdCheckSleep();
  checkBtn();
}


void checkBtn(){
  if (btnWasPressed){
    if (digitalRead(BUTTON_PIN)){
      //button no longer pressed
      unsigned long timePassed = millis() - timeOfPress;
      if (timePassed >= LONG_PRESS_TIME){
        buttonPress(LONG_PRESS);
      } else if (timePassed >= SHORT_PRESS_TIME){
        buttonPress(SHORT_PRESS);
      } 
    }
  }

  //Button release debounce
  if (btnDeactivated){
    if (timePassed(&btnTimeOfLastDeactivation, BTN_DEACTIVATION_TIME_MS)){
      //Debounce time for button realeased has passed
      btnDeactivated = false;
      attachInterrupt(BUTTON_PIN, buttonISR, FALLING);
    }
  }
  
}
//Interrupt that will be called from button press
void IRAM_ATTR buttonISR(){
  if (!btnWasPressed){
    btnWasPressed = true;
    //Save time for debounce and destinguishing between long and short press
    timeOfPress = millis();
    //Stop interrupt for debouncing when the button is realeased
    detachInterrupt(BUTTON_PIN);
  }
}

void buttonPress(byte pressType){
  btnWasPressed = false;
  btnDeactivated = true;
  btnTimeOfLastDeactivation = millis();
  debugPrint("BTN ");
  //To know when to sleep the display
  lastButtonPress = millis();
  if (pressType == SHORT_PRESS){
    debugPrintln("SHORT");
    if (lcdOn){
      screenNumber++;
      if (screenNumber > MAX_SCREEN_NUMBER){
        screenNumber = 1;
      }
    } else {
      lcdTurnOn();
      screenNumber = 1;
    }
    updateLCD();
  } else if (pressType == LONG_PRESS) {
    debugPrintln("LONG");
    lcdTurnOff();
  }
}

//Sleep LCD if the button has not been pressed
void lcdCheckSleep(){
  if (lcdOn){
    if (timePassed(&lastButtonPress, LCD_SLEEP_DELAY_MS)){
      lcdTurnOff();
    }
  }
}

void lcdTurnOn(){
  lcd.begin(16 ,2);
  lcd.display();
  digitalWrite(PIN_BACKLIGHT, HIGH);
  lcdOn = true;
}

void lcdTurnOff(){
  lcd.noDisplay();
  digitalWrite(PIN_BACKLIGHT, LOW);
  lcdOn = false;
}

//Show different readings on the screen
void updateLCD (){
  if (timePassed(&lastLCDupdate, LCD_UPDATE_FRQ_MS)){
    switch (screenNumber)
    {
    case 1:
      //CO2, t, TV, RH
      lcd.clear();
      lcd.print("CO2:");
      lcd.print(arrCO2_TVOC[0]);
      lcd.setCursor(8, 0);
      lcd.print("t:");
      lcd.print(temperature);
      lcd.setCursor(0, 1);
      lcd.print("TV:");
      lcd.print(arrCO2_TVOC[1]);
      lcd.setCursor(8, 1);
      lcd.print("RH:");
      lcd.print(humidity);
      break;
    case 2:
      //Pressure, t2
      lcd.clear();
      lcd.print("P:");
      lcd.print(pressure);
      lcd.setCursor(0, 1);
      lcd.print("T2:");
      lcd.print(temperature2);
      break;
    case 3:
      //Internet connection and IP
      lcd.clear();
      if (WiFi.status() == WL_CONNECTED){
        lcd.print("WiFi OK");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.localIP());
      } else {
        lcd.print("WiFi FAULT!");
      }
      break;
    default:
      Serial.println("Invalid screen number");
      break;
    }
  }
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
      Serial.println("Clearing flash");
      Serial.println("Recalibration will be executed");
      clearBaseValues();
      break;
    case 'r':
      debugPrintln("Using stored values");
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
  if (calibFunc == 2){
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
  } else {
    debugPrintln("Skipping initial values");
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
    startBaseValueStoreCntr++;
    if (startBaseValueStoreCntr > STORE_BASE_VALUES_START){
      //delay the first read as it is recommended by the DS
      startBaseValueStoreCntr = STORE_BASE_VALUES_START;
      debugPrintln("Reading base values");
      uint16_t  bLine[2];
      CO2s.getBaseline(bLine);
      debugPrint("eCO2 base:"); debugPrintln((String)bLine[0]);
      debugPrint("TVOC base:"); debugPrintln((String)bLine[1]);
      byte CO2byte1 = bLine[0] >> 8;
      byte CO2byte2 = bLine[0];
      byte TVOCbyte1 = bLine[1] >> 8;
      byte TVOCbyte2 = bLine[1];
      EEPROM.write(EEPADD_BASE_EXISTS, 2);
      EEPROM.write(EEPADD_CO2_1, CO2byte1);
      EEPROM.write(EEPADD_CO2_2, CO2byte2);
      EEPROM.write(EEPADD_TVOC_1, TVOCbyte1);
      EEPROM.write(EEPADD_TVOC_2, TVOCbyte2);
      EEPROM.commit();
      debugPrintln("Base values saved");
    }
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

//Read from BMP sensor
void measurePressureTemp(){
  if (timePassed(&lastPressureRead, READ_F_BMP_P_T_MS)){
    sensors_event_t temp_event, pressure_event;
    bool tempReadSuccessfull = bmp_temp->getEvent(&temp_event);
    bool pressureReadSuccessfull = bmp_pressure->getEvent(&pressure_event);
    if (tempReadSuccessfull && pressureReadSuccessfull){
      bmpReadValid = true;
      Serial.print(F("T = "));
      Serial.print(temp_event.temperature);
      Serial.println(" C");

      Serial.print(F("P = "));
      Serial.print(pressure_event.pressure);
      Serial.println(" hPa");
      temperature2 = temp_event.temperature;
      pressure = pressure_event.pressure;
    } else {
      bmpReadValid = false;
      Serial.println("BMP measurement failed");
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

//Returns rtue if time has passed since the previous time the event happened
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
  //for WiFi connection animation
  byte wifiAnimConStep = 0;
  lcd.clear();
  lcd.print("Init WiFi");
  Serial.print("Connecting to WiFi");
  Serial.println(Secrets::SSID);
  WiFi.begin(Secrets::SSID, Secrets::PASSWORD);
  while (!connectTimeOut && !wifiConnected){
    if ((millis() - writeCharTime) > 500){
      //connecting indicator
      Serial.print(".");
      writeCharTime = millis();
      lcdWifiAnim(wifiAnimConStep);
      wifiAnimConStep++;
      if (wifiAnimConStep > 4){
        wifiAnimConStep = 1;
      }
    }
    if (WiFi.status() == WL_CONNECTED){
      //Connection successful
      wifiConnected = true;
      lcd.setCursor(0, 0);
      lcd.print("Wifi success");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("MAC address: "); 
      Serial.println(WiFi.macAddress());
    }
    if (millis() - connectWifiStartTime > MAX_WIFI_CONNECT_TIME_MS){
      //timed out of wifi connection
      if (!wifiConnected){
        connectTimeOut = true;
        Serial.println("");
        Serial.println("WiFi connect time out");
        lcd.setCursor(0, 0);
        lcd.print("Wifi connection");
        lcd.setCursor(0, 1);
        lcd.print("failed");
      }
    }
  }
  //allow the user to see the wifi status on the LCD
  delay(1000);
}

void lcdWifiAnim (int step){
  lcd.setCursor(0,1);
  switch (step)
  {
  case 1:
    lcd.print(".  ");
    break;
  case 2:
    lcd.print(".. ");
    break;
  case 3:
    lcd.print("...");
    break;
  case 4:
    lcd.print("   ");
    break;
  default:
    break;
  }
}

void setupFireBase(){
  Firebase.begin(Secrets::FIREBASE_HOST, Secrets::FIREBASE_AUTH);
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
    if (bmpReadValid){
      json2.set("/pressure", pressure);
      json2.set("/temperature2", temperature2);
    }
    //Push the data to a random name
    Firebase.push(firebaseData, "/Log/",json2);
    //get the key
    String key = firebaseData.pushName();
    //Set time stamp in the just made log
    Firebase.setTimestamp(firebaseData, "/Log/" + key+ "/Time");
    //logNr++;
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
    if (bmpReadValid){
      json.set("/temperature2", temperature2);
      json.set("/pressure", pressure);
    }
    Firebase.updateNode(firebaseData,"/SensorStation",json);
    Firebase.setTimestamp(firebaseData, "/SensorStation/LastUpdate");    
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