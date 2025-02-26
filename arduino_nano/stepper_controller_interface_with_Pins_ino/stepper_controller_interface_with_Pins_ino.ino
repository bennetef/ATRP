/**
 * @file arduino_sensor_skript.ino
 * @brief This program awaits commands via an i2c connection and sends appropriate inputs to the stepper controller
 * @date 2022-03-17 
 */
// Wire.h is used for sending data
#include <Wire.h>

// CONSTANTS
#define DRIVER_PUL 7       // motor driver pin
#define DRIVER_DIR 8       // motor driver pin
#define PULSE_DURATION 200
#define STEER_RES 162.5        // steering resolution (number of steps to steer per command) (162,5 - 1 degree)

#define RIGHT 0x0
#define LEFT 0x1

// endstop pins
int leftEndstop = A2; 
int rightEndstop = A3;

// steering pins
int leftSteeringPin = A6;
int rightSteeringPin = A7;

// i2c Address (check if arduino is detected using "i2cdetect -y -r 1")
int i2cAddress = 0x60;

// indicates maximum number of steps to left and right of middle position
int maxNumSteps = 0;
int step = 0; // the current step (positive is right, negative is left)

// variable to check for faults
bool faultDetected = false;

/**
 * @brief Setting up arduino
 * 
 * Define pinmodes and set up i2c connection
 */
void setup() {
  Serial.begin(9600);
  
  // i2c connection set up
  Wire.begin(i2cAddress);
  //Wire.onRequest(readSteering);
  
  // Pin set up
  pinMode(DRIVER_PUL, OUTPUT);
  pinMode(DRIVER_DIR, OUTPUT);
  
  pinMode(leftEndstop, INPUT);
  pinMode(rightEndstop, INPUT);

  pinMode(leftSteeringPin, INPUT);
  pinMode(rightSteeringPin, INPUT);

  // home steering
  delay(3000);
  maxNumSteps = homing();   // +/-2600
  //move();
  delay(3000);
}

void loop() {
  // if endstops are pressed after hooming completed, fault detected
  /**
  if (digitalRead(leftEndstop) == HIGH){
    faultDetected = true;

    delay(1000);

    for(int i = 0; i <= STEER_RES; i++){     
        digitalWrite(DRIVER_DIR,RIGHT);
        digitalWrite(DRIVER_PUL,HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL,LOW);
        delayMicroseconds(PULSE_DURATION);
    }

    delay(1000);
    faultDetected = false;
  }

  if (digitalRead(rightEndstop) == HIGH){
    faultDetected = true;

    delay(1000);

  	for(int i = 0; i <= STEER_RES; i++){     
        digitalWrite(DRIVER_DIR,LEFT);
        digitalWrite(DRIVER_PUL,HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL,LOW);
        delayMicroseconds(PULSE_DURATION);
    }

    delay(1000);
    faultDetected = false;
  }
**/
  readSteering();
}

/**
 * @brief Functions homes stepper motor so that steering angle is zero. Returns the number of steps possible in each direction.
 * Also positions steering to middle position
 */
int homing(){
  int counter = 0;
  step = 0;

  // wait until right endstop is pressed
  digitalWrite(DRIVER_DIR,RIGHT);
  while(digitalRead(rightEndstop) == LOW){
    digitalWrite(DRIVER_PUL,HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(DRIVER_PUL,LOW);
    delayMicroseconds(PULSE_DURATION);
  }

  delay(1000);

  // now wait until left endstop is pressed
  digitalWrite(DRIVER_DIR,LEFT);
  while(digitalRead(leftEndstop) == LOW){
    digitalWrite(DRIVER_PUL,HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(DRIVER_PUL,LOW);
    delayMicroseconds(PULSE_DURATION);

    // count number of steps
    counter++;
  }

  int middleSteps = (int) counter / 2;
  //Serial.println(middleSteps);

  delay(1000);

  // return to middle position
  digitalWrite(DRIVER_DIR,RIGHT);
  for(int i = 0; i <= middleSteps; i++){     
      digitalWrite(DRIVER_PUL,HIGH);
      delayMicroseconds(PULSE_DURATION);
      digitalWrite(DRIVER_PUL,LOW);
      delayMicroseconds(PULSE_DURATION);
  }

  // return floor of counter / 2
  return middleSteps;
}

// function that executes whenever data is received from master
void readSteering()
{
  //if (!faultDetected){
    //Serial.println(analogRead(rightSteeringPin));

    //char c = Wire.read(); // receive byte as a character
    // 400 analog read approximates roughly 2V for 10bit resolution
    if (analogRead(rightSteeringPin) > 400 && step < maxNumSteps - 2*STEER_RES){
      // move to the right
      Serial.println("Moving Right");
      digitalWrite(DRIVER_DIR,RIGHT);
      for(int i = 0; i <= STEER_RES; i++){     
        digitalWrite(DRIVER_PUL,HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL,LOW);
        delayMicroseconds(PULSE_DURATION);
      }
      
      // increase steps
      step += STEER_RES;
    // 400 analog read approximates roughly 2V for 10 bit resolution
    } else if (analogRead(leftSteeringPin) > 400 && step > - maxNumSteps + 2*STEER_RES) {
      // move to the left
      Serial.println("Moving Left");
      digitalWrite(DRIVER_DIR,LEFT);
      for(int i = 0; i <= STEER_RES; i++){    
        digitalWrite(DRIVER_PUL,HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL,LOW);
        delayMicroseconds(PULSE_DURATION);
      }
      // decrease steps
      step -= STEER_RES;
    } 
    Serial.println(step);
    Wire.write(step);
    //Serial.println(c); 
  //}
}