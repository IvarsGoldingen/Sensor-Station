//This needs to be included in the library so it can access standard arduino functions
#include "Arduino.h"
#include "ButtonIB.h"

const byte ButtonIB::SHORT_PRESS = 1;
const byte ButtonIB::LONG_PRESS = 2;

//Constructor 1
ButtonIB::ButtonIB(byte pin, void(*cbPressed)(byte)){
  btnPressedCb = cbPressed;
  button_pin = pin;
  pinMode(button_pin, INPUT_PULLUP);
}

void ButtonIB::loop(){
  checkButtonTrigger();
}

/*
 * Returns true when button is pressed
 */
void ButtonIB::checkButtonTrigger(){
  if(!digitalRead(button_pin)){
    if (!btnWasPressed){
      //button was not pressed before, save time of press
      timeOfPress = millis();
      btnWasPressed = true;
    }    
  } else {
    //button not pressed, check if it was previously
    unsigned int timePassed = millis() - timeOfPress;
    if (btnWasPressed){
      //button was pressed before, check if debounce time had elapsed
      if(timePassed >= LONG_PRESS_TIME){
        btnPressedCb(LONG_PRESS);
      } else if (timePassed >= SHORT_PRESS_TIME){
        btnPressedCb(SHORT_PRESS);
      }
    } else {
      btnWasPressed = false;
    }
  }
}