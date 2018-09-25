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

int RPM = 200;
int ROTATIONS = 1;
int INTERVAL = 5000; // in mS
int deg = ROTATIONS*360;
String incomingString = "";                         // a string for incoming text

/*************************************************************/


void displayParameters()
{

  Serial.println ("Current parameter values:");
  Serial.print ("RPM = ");
  Serial.println (RPM);
  Serial.print ("ROTATIONS = ");
  Serial.println (ROTATIONS);
  Serial.print ("Degrees = ");
  Serial.println (deg);
  Serial.print ("INTERVAL = ");
  Serial.println (INTERVAL);

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
      RPM = values[0].toInt();
      ROTATIONS = values[1].toInt();
      INTERVAL = values[2].toInt();


      String dir_str = values[3];
      dir_str.toLowerCase();
      //Serial.println(str.indexOf("clock"));
      //Serial.println(str);

      if ( dir_str.indexOf("clock") > -1 )
        ;
      else if ( dir_str.indexOf("anti") > -1 )
        ROTATIONS = 0-ROTATIONS;
      /**************************************************************************/


      /*********** In case of wrong input initializing default values ***********/
      if ( RPM <= 0 or ROTATIONS == 0 or INTERVAL < 0) {
        // Assigning default values
        Serial.println ('Assigning default values');
        RPM=200; // Rotations per minute (Speed of motor)
        ROTATIONS=1; // Number of rotations
        INTERVAL = 5000; // Delay after n number of rotations (mS) (Where n = ROTATIONS)
      }
      /**************************************************************************/

      deg = ROTATIONS*360;
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

    if (Serial.available())
      updateParameters();

    if (mySerial.available())
      updateParameters();

    delay(INTERVAL);



    // In 1:32 microstepping mode, one revolution takes 8 times as many microsteps
    //stepper.move(32 * MOTOR_STEPS);    // forward revolution
    //stepper.move(-32 * MOTOR_STEPS);   // reverse revolution

    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    stepper.rotate(deg);
    //stepper.rotate(-360);

}
