#include "Arduino.h"
#include "CO2_sens.h"

//Constructor
CO2_sens::CO2_sens(){

}

void CO2_sens::setup(TwoWire *I2C){
    // Serial.println(F("DHTxx test!"));
    Serial.println("SGP30 start setup");
    sensor = new Adafruit_SGP30();
    if (!sensor->begin(I2C)){
        Serial.println("SGP not found");
    } else {
        Serial.print("Found SGP30 serial #");
        Serial.print(sensor->serialnumber[0], HEX);
        Serial.print(sensor->serialnumber[1], HEX);
        Serial.println(sensor->serialnumber[2], HEX);
    }
}

bool CO2_sens::measure(uint16_t arr[]){
    if (! sensor->IAQmeasure()) {
        return false;
    }
    //write measured values in the passed array
    arr[0] = sensor->eCO2;
    arr[1] = sensor->TVOC;
    return true;
}

bool CO2_sens::measureRaw(uint16_t arr[]){
    if (! sensor->IAQmeasureRaw()) {
        return false;
    }
    //write measured values in the passed array
    arr[0] = sensor->rawH2;
    arr[1] = sensor->rawEthanol;
    return true;
}

bool CO2_sens::getBaseline(uint16_t arr[]){
    if (! sensor->getIAQBaseline(&arr[0], &arr[1])) {
        return false;
    }
    return true;
}

bool CO2_sens::setHumidity(uint32_t humidity){
    return sensor->setHumidity(humidity);
}

bool CO2_sens::setBaseLine(uint16_t CO2Base, uint16_t TVOCBase){
    if (sensor->setIAQBaseline(CO2Base, TVOCBase)){
        //successful set
        return true;
    } else {
        //failed to set
        return false;
    }
}



