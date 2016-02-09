/*
 * @author("Raymond Blum" <raymond@insanegiantrobots.com>)
 * Targeted for an Arduino Uno
 *
 *******************
 switbit by Raymond Blum is licensed under a
 GNU LESSER GENERAL PUBLIC LICENSE Version 3
 *******************
*/

#define NO_DEBUG

#include <Wire.h>
#include <SparkFun_APDS9960.h>

// Pins
#define APDS9960_INT    2 // Needs to be an interrupt pin
#define LED  13
#define IND1 A2
#define IND2 A3
#define DIP1 5
#define DIP2 6

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int isr_flag = 0;

int dip1State, dip2State;

void indicate(int pin) {
  digitalWrite(pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);               // wait for a second
  digitalWrite(pin, LOW);    // turn the LED off by making the voltage LOW
  }

void indicateAll() {
  digitalWrite(IND1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(IND2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);               // wait for a second
  digitalWrite(IND1, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(IND2, LOW);   // turn the LED on (HIGH is the voltage level)
}

void blink() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
}

void setup() {
  // Set interrupt pin as input
  pinMode(APDS9960_INT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(IND1, OUTPUT);
  pinMode(IND2, OUTPUT);
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);
  blink();
  readDips();
  if (dip1State == HIGH) {
    indicate(IND1);
  }
 if (dip2State == HIGH) {
    indicate(IND2);
  }

  // Initialize Serial port
  #ifdef _DEBUG
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - GestureTest"));
  Serial.println(F("--------------------------------"));
  #endif
  // Initialize interrupt service routine
  attachInterrupt(digitalPinToInterrupt(APDS9960_INT), interruptRoutine, FALLING);
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    #ifdef _DEBUG
    Serial.println(F("APDS-9960 initialization complete"));
    #endif
  } else {
    #ifdef _DEBUG
    Serial.println(F("Something went wrong during APDS-9960 init!"));
    #endif
  }
  
  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    #ifdef _DEBUG
    Serial.println(F("Gesture sensor is now running"));
    #endif
  } else {
    #ifdef _DEBUG
    Serial.println(F("Something went wrong during gesture sensor init!"));
    #endif
  }
}

void readDips() {
  dip1State = digitalRead(DIP1);
  dip2State = digitalRead(DIP2);
}

void loop() {
  readDips();
  if( isr_flag == 1 ) {
    detachInterrupt(0);
    handleGesture();
    isr_flag = 0;
    attachInterrupt(digitalPinToInterrupt(APDS9960_INT), interruptRoutine, FALLING);
  }
}

void interruptRoutine() {
  isr_flag = 1;
}

void handleGesture() {
    if ( apds.isGestureAvailable() ) {
      switch ( apds.readGesture() ) {
        case DIR_UP:
          if (dip1State == HIGH) {
            #ifdef _DEBUG
            Serial.println("UP");
            #endif
            indicate(IND1);
          }
          break;
        case DIR_DOWN:
          if (dip1State == HIGH) {
            #ifdef _DEBUG
            Serial.println("DOWN");
            #endif
            indicate(IND2);
          }
          break;
        case DIR_LEFT:
          if (dip1State == HIGH) {
            #ifdef _DEBUG
            Serial.println("LEFT");
            #endif
            indicateAll();
          }
          break;
        case DIR_RIGHT:
          if (dip1State == HIGH) {
            #ifdef _DEBUG
            Serial.println("RIGHT");
            #endif
            indicateAll();
          }
          break;
        case DIR_NEAR:
          if (dip2State == HIGH) {
            #ifdef _DEBUG
            Serial.println("NEAR");
            #endif
            indicate(IND1);
            indicate(IND2);
          }
          break;
        case DIR_FAR:
          if (dip2State == HIGH) {
            #ifdef _DEBUG
            Serial.println("FAR");
            #endif
            indicate(IND2);
            indicate(IND1);
          }
          break;
        default:
          #ifdef _DEBUG
          Serial.println("NONE");
          #endif
          ;
      }
    }
  }
