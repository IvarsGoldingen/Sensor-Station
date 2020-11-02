//This prevents problems if somehow this calls was implemented twice in the same scetch
#ifndef ButtonIB_h
#define ButtonIB_h

/*
 * Class for cntrolling inputs, specifically for buttons
 * connect button to GND, because pullup used on input
 */

 //This needs to be included in the library so it can access standard arduino functions
#include "Arduino.h"
//Minimum lenght of button presses to be registered
#define SHORT_PRESS_TIME 40
#define LONG_PRESS_TIME 600

class ButtonIB{
  private:
    //Class member variables
    //Input pin of the button
    byte button_pin;
    //stores the last time the button press was registered. Used for debouncing purposes
    long timeOfPress = 0;
    //Check if the button was pressed in the last loop
    bool btnWasPressed = false;
    /*
     * Function called in each loop to check if the button has been pressed
     * Returns true when the button has been let go of
     */
     void checkButtonTrigger();
     //A pointer to a function in the main scetch. Triggered when button is pressed
    void (*btnPressedCb)(byte) = 0;
  public:
    //Bytes that will be sent to the main scketch to identify if the press was long or short
    const static byte SHORT_PRESS = 1;
    const static byte LONG_PRESS = 2;
    /*
    *A constructor with onoly single press callback
    *pin = the input pin which is connected to the button
    **cbPressed = callback to the main function when button has been pressed
    */
    ButtonIB(byte pin, void(*cbPressed)(byte));
    /*
     * Loop function of the class
    */ 
    void loop();
};

#endif
