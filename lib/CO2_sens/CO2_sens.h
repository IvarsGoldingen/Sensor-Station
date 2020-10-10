//This prevents problems if somehow this calls was implemented twice in the same scetch
#ifndef CO2_sens_h
#define CO2_sens_h

/*
 * Class for using CO2 sensor SGP30
 * implements adafruit library
 */

 //This needs to be included in the library so it can access standard arduino functions
#include "Arduino.h"
//Adafruit lib
#include <Adafruit_SGP30.h>
//I2C lib
#include <Wire.h>

class CO2_sens{
  private:
    //Pointer to adafrui library objects
    Adafruit_SGP30* sensor;
    //counter for calibration
    int counter = 0;
    

  public:
    /*********************************************************************************
     * A constructor for the class
     * 
    *********************************************************************************/
    CO2_sens();

    //Setup function of the sensor
    //Requires a pointer to I2C interface
    void setup(TwoWire *I2C);
    /*************************************************************************
     * Measure raw H2 and ethalon
     * returns true if measured scessfully
     * gets passed an array where values are put
     * arr[0] = H2
     * arr[1] = ethalon
     ************************************************************************/
    bool measureRaw(uint16_t arr[]);
    /*************************************************************************
     * Measure TVOC and CO2
     * returns true if measured scessfully
     * gets passed an array where values are put
     * arr[0] = CO2 ppm
     * arr[1] = TVOC ppb
     ************************************************************************/
    bool measure(uint16_t arr[]);
    /*************************************************************************
     * Get Baseline
     * returns true if measured scessfully
     * gets passed an array where values are put
     * arr[0] = 
     * arr[1] = 
     ************************************************************************/
    bool getBaseline(uint16_t arr[]);
    //Set humidity for more precise measurements
    //returns true if successful
    bool setHumidity(uint32_t);
    //set baseline values in sensor
    bool setBaseLine(uint16_t CO2Base, uint16_t TVOCBase);
};

#endif