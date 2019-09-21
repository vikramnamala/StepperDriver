/*
 * Microstepping demo
 *
 * This requires that microstep control pins be connected in addition to STEP,DIR
 *
 * Copyright (C)2015 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>

#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX, TX

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
/* Old code
#define MOTOR_STEPS 200
#define RPM 120

#define DIR 8
#define STEP 9
#define ENABLE 13 // optional (just delete ENABLE from everywhere if not used)
*/

#define MOTOR_STEPS 200
// #define RPM 120

#define DIR 6
#define STEP 3
#define ENABLE 8 // optional (just delete ENABLE from everywhere if not used)


/*
 * Choose one of the sections below that match your board
 */

// #include "DRV8834.h"
// #define M0 10
// #define M1 11
// DRV8834 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, M0, M1);

// #include "A4988.h"
// #define MS1 10
// #define MS2 11
// #define MS3 12
// A4988 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, MS1, MS2, MS3);


#include "DRV8825.h"
// #define MODE0 10
// #define MODE1 11
// #define MODE2 12
// DRV8825 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, MODE0, MODE1, MODE2);
DRV8825 stepper(MOTOR_STEPS, DIR, STEP, ENABLE);


// #include "DRV8880.h"
// #define M0 10
// #define M1 11
// #define TRQ0 6
// #define TRQ1 7
// DRV8880 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, M0, M1, TRQ0, TRQ1);

// #include "BasicStepperDriver.h" // generic
// BasicStepperDriver stepper(DIR, STEP);



/******************** GLOBAL DECLARATIONS ********************/

float RPM = 12.0;
float ROT_DURATION = 2000.0; // in mS
float deg = ((ROT_DURATION*RPM*360.0)/(1000.0*60.0));
unsigned long SHUTTER = 10000; // in mS
// float deg1 = deg*360;
int COMPLETED_CYCLES = 0;
float COMPLETED_DEG = 0.0;
float MAX_ROT = 20.0;
float MAX_DEG = MAX_ROT*360.0;
int MAX_CYCLES = MAX_DEG/deg;
String incomingString = "";                         // a string for incoming text
unsigned long previousMillis = 0;        // will store last time a function was updated

// RPS = RPM/60 = 0.2

/*************************************************************/


void displayParameters()
{
  Serial.println ("Current parameter values:");
  Serial.print ("RPM = ");
  Serial.println (RPM);
  Serial.print ("DURATION OF A ROTATION CYCLE = ");
  Serial.println (ROT_DURATION);
  Serial.print ("deg IN A CYCLE = ");
  Serial.println (deg);
  Serial.print ("Degrees = ");
  Serial.println (deg);
  Serial.print ("SHUTTER DURATION = ");
  Serial.println (SHUTTER);
  Serial.print ("TOTAL DURATION OF EACH CYCLE = ");
  Serial.println (ROT_DURATION + SHUTTER);
  Serial.print ("TOTAL NUMBER OF POSSIBLE CYCLES = ");
  Serial.println (MAX_CYCLES);
  Serial.print ("TOTAL DURATION OF ENTIRE SLIDE = ");
  Serial.println ((ROT_DURATION + SHUTTER)*MAX_CYCLES);
}


void updateParameters()
{
  while (Serial.available()) {
    incomingString = Serial.readString(); // read the incoming byte:
  }

  while (mySerial.available()) {
    incomingString = mySerial.readString(); // read the incoming byte:
  }

  Serial.print(" I received:");
  incomingString.trim();
  Serial.println(incomingString);


  String values[ 10 ] ; // values is an array of 10 strings


// Format of string input ~rpm:rot_duration:shutter_interval:dir|
  if (incomingString.startsWith("~") and incomingString.endsWith("|")) {

      stepper.disable();

      incomingString = incomingString.substring(1, incomingString.length()-1);

      /************ Spliting incoming string and loading into array 'values' ************/
      int i = 0;
      for ( int j = 0; j < 10 and i != -1; ++j ) {
        i = incomingString.indexOf(':');
        values[j] = incomingString.substring( 0, i );
        incomingString = incomingString.substring( i+1 , incomingString.length() );
      }
      /**********************************************************************************/



      // output each array element's value
      /*
      for ( int j = 0; j < 10 and values[j] != "" ; ++j ) {
          Serial.print (values[j]) ;
          Serial.println (j) ;
      }
      */

      /**************** Initializing parameter using array **********************/
      RPM = values[0].toFloat();
      ROT_DURATION = values[1].toFloat();
      SHUTTER = values[2].toFloat();

      deg = (ROT_DURATION*RPM*360.0)/(1000.0*60.0);

      String dir_str = values[3];
      dir_str.toLowerCase();
      //Serial.println(str.indexOf("clock"));
      //Serial.println(str);

      if ( dir_str.indexOf("clock") > -1 )
        deg = deg;
      else if ( dir_str.indexOf("anti") > -1 )
        deg = 0.0-deg;
      /**************************************************************************/


      /*********** In case of wrong input initializing default values ***********/
      if ( RPM <= 0.0 or ROT_DURATION < 0.0 or SHUTTER < 0) {
        // Assigning default values
        Serial.println ('Assigning default values');

        RPM=12.0; // deg per minute (Speed of motor)
        ROT_DURATION = 2000.0; // in mS
        deg = ((ROT_DURATION*RPM*360.0)/(1000.0*60.0)); // Number of deg
        SHUTTER = 10000; // Delay after n number of deg (mS) (Where n = deg)            
      }
      /**************************************************************************/

      // deg1 = deg*360;
      COMPLETED_CYCLES = 0;
      COMPLETED_DEG = 0.0;
      MAX_ROT = 20.0;
      MAX_DEG = MAX_ROT*360.0;
      MAX_CYCLES = MAX_DEG/abs(deg);
      stepper.begin(RPM);
      stepper.enable();
      stepper.setMicrostep(32);
   }

   displayParameters();
}


void setup() {

    Serial.begin(115200);               // opens serial port, sets data rate to 115200 bps

    mySerial.begin(9600);

    displayParameters();

    if (Serial.available())
      updateParameters();

    if (mySerial.available())
      updateParameters();

    stepper.begin(RPM);
    stepper.enable();

    // set current level (for DRV8880 only).
    // Valid percent values are 25, 50, 75 or 100.
    // stepper.setCurrent(100);

    /*
     * Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
     * Mode 1 is full speed.
     * Mode 32 is 32 microsteps per step.
     * The motor should rotate just as fast (at the set RPM),
     * but movement precision is increased, which may become visually apparent at lower RPMs.
     */
    stepper.setMicrostep(32);   // Set microstep mode to 1:32

}

void loop() {

    unsigned long currentMillis = millis();

    if (Serial.available())
      updateParameters();

    if (mySerial.available())
      updateParameters();

    //delay(SHUTTER);

    if (currentMillis - previousMillis >= (SHUTTER + ROT_DURATION) and COMPLETED_CYCLES < MAX_CYCLES) {
      // save the last time you ran the function
      previousMillis = currentMillis;
      stepper.rotate(deg);
      COMPLETED_CYCLES = COMPLETED_CYCLES + 1;
      COMPLETED_DEG = COMPLETED_DEG + deg;
      Serial.print ("COMPLETED_CYCLES = ");
      Serial.println (COMPLETED_CYCLES);
      Serial.print ("COMPLETED_DEG = ");
      Serial.println (COMPLETED_DEG);
    }


    // In 1:32 microstepping mode, one revolution takes 8 times as many microsteps
    //stepper.move(32 * MOTOR_STEPS);    // forward revolution
    //stepper.move(-32 * MOTOR_STEPS);   // reverse revolution

    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    //stepper.rotate(deg1);
    //stepper.rotate(-360);

}
