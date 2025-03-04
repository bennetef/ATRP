#include <Wire.h>

// Constants
#define DRIVER_PUL 7
#define DRIVER_DIR 8
#define PULSE_DURATION 200
#define RIGHT 0
#define LEFT 1

// endstop pins
int leftEndstop = A2; 
int rightEndstop = A3;

// steering pins
int leftSteeringPin = A6;
int rightSteeringPin = A7;

const float STEPS_PER_DEGREE = 162.5;

// State variables
int targetSteps = 0;
int maxNumSteps = 2600;  // Example max - you can change this after homing
int currentSteps = 0;
int steps = 0;

// Command buffer
volatile int pendingDirection = -1;
volatile int pendingDegrees = 0;
volatile bool flag = false;

// Previous command values
int previousDirection = -1;
int previousDegrees = 0;

// Setup
void setup() {
  Serial.begin(9600);
  Wire.begin(0x60);
  Wire.onReceive(receiveSteeringCommand);

  pinMode(DRIVER_PUL, OUTPUT);
  pinMode(DRIVER_DIR, OUTPUT);

  pinMode(leftEndstop, INPUT);
  pinMode(rightEndstop, INPUT);

  pinMode(leftSteeringPin, INPUT);
  pinMode(rightSteeringPin, INPUT);

  delay(3000);
  maxNumSteps = homing();
  delay(3000);
}

void loop() {
  if (flag == true) {
    // Only update targetSteps if the command has changed
    if (pendingDegrees != previousDegrees || pendingDirection != previousDirection) {
      if(pendingDirection == 0){
        targetSteps = -pendingDegrees * STEPS_PER_DEGREE;
      }else{
        targetSteps = pendingDegrees * STEPS_PER_DEGREE;
      }
      previousDegrees = pendingDegrees;
      previousDirection = pendingDirection;
    }
    flag = false;
  }

  if (abs(targetSteps - currentSteps) > 100){
    moveChunk(pendingDirection);
  }

  readSteering();
}

/**
 * @brief Functions homes stepper motor so that steering angle is zero. Returns the number of steps possible in each direction.
 * Also positions steering to middle position
 */
int homing(){
  int counter = 0;
  currentSteps = 0;

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
  Serial.println("Homing Finished");
  return middleSteps;
}

// Receive and store steering command (interrupt handler)
void receiveSteeringCommand(int numBytes) {
  if(numBytes == 3){
 
      int trash = Wire.read();   
      pendingDirection = Wire.read();     // 0 = left, 1 = right
      pendingDegrees = Wire.read();   // Integer degrees

      //Serial.println("Di");
      //Serial.println(pendingDirection);
      //Serial.println("De");
      //Serial.println(pendingDegrees);

      flag = true;
  } else{
    Wire.read();
  }
}

// Move a single chunk (1 degree = 162.5 steps)
bool moveChunk(int direction) {
    int chunk = STEPS_PER_DEGREE + 0.5;
    int stepDir = (direction > 0) ? RIGHT : LEFT;

    // Ensure the motor is stopped before changing directions
    if (direction != previousDirection) {
        delay(100);  // Small delay to allow the motor controller to handle the change in direction
    }

    if (stepDir == RIGHT && currentSteps + chunk > maxNumSteps) {
        Serial.println("Right limit reached");
        return false;  // Hit right limit
    }
    if (stepDir == LEFT && currentSteps - chunk < -maxNumSteps) {
        Serial.println("Left limit reached");
        return false;  // Hit left limit
    }

    digitalWrite(DRIVER_DIR, stepDir);

    for (int i = 0; i < chunk; i++) {
        digitalWrite(DRIVER_PUL, HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL, LOW);
        delayMicroseconds(PULSE_DURATION);
    }

    if (stepDir == RIGHT) {
        currentSteps += chunk;
    } else {
        currentSteps -= chunk;
    }

    Serial.println(currentSteps);

    return true;
}
void readSteering()
{
  //if (!faultDetected){
    //Serial.println(analogRead(rightSteeringPin));

    //char c = Wire.read(); // receive byte as a character
    // 400 analog read approximates roughly 2V for 10bit resolution
    if (analogRead(rightSteeringPin) > 700 && steps < maxNumSteps - 2*STEPS_PER_DEGREE){
      // move to the right
      Serial.println("Moving Right");
      //Serial.println(analogRead(rightSteeringPin));
      digitalWrite(DRIVER_DIR,RIGHT);
      for(int i = 0; i <= STEPS_PER_DEGREE+0.5; i++){     
        digitalWrite(DRIVER_PUL,HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL,LOW);
        delayMicroseconds(PULSE_DURATION);
      }
      
      // increase steps
      steps += STEPS_PER_DEGREE;
    // 400 analog read approximates roughly 2V for 10 bit resolution
    } else if (analogRead(leftSteeringPin) > 400 && steps > - maxNumSteps + 2*STEPS_PER_DEGREE) {
      // move to the left
      Serial.println("Moving Left");
      digitalWrite(DRIVER_DIR,LEFT);
      for(int i = 0; i <= STEPS_PER_DEGREE+0.5; i++){    
        digitalWrite(DRIVER_PUL,HIGH);
        delayMicroseconds(PULSE_DURATION);
        digitalWrite(DRIVER_PUL,LOW);
        delayMicroseconds(PULSE_DURATION);
      }
      // decrease steps
      steps -= STEPS_PER_DEGREE;
    } 
    //Serial.println(steps);
    //Wire.write(step);
    //Serial.println(c);
  //}
}
