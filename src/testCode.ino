#include <AccelStepper.h>
#include <Servo.h>

#define debug true

// ROLL CONSTANTS
#define dirPin 5
#define stepPin 3
#define motorInterfaceType 1
#define rollEnable 2
#define microStepCoef 8

// PITCH CONSTANTS
#define pitchPin1 10
#define pitchPin2 11
int pitchCurrentSpeed;

// GRIP CONSTANTS
#define gripPinA 8
#define gripPinB 9

// CONTROLLER PINS
#define gripClosePin 12
#define gripOpenPin 13
#define rollControlPin A0
#define pitchControlPin A1
#define potDeadBand 30

int pitchCal;
int rollCal;

// MOTOR DEFINITIONS
AccelStepper rollStepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
Servo pitchServo1;
Servo pitchServo2;

void setup()
{
  Serial.begin(9600);

  // pinMode(dirPin, OUTPUT);
  // pinMode(stepPin, OUTPUT);
  pinMode(pitchPin1, OUTPUT);
  pinMode(pitchPin2, OUTPUT);

  // Initialize roll
  rollStepper.setMaxSpeed(700 * microStepCoef);
  rollStepper.setAcceleration(2000 * microStepCoef);
  rollStepper.moveTo(0);
  zeroRoll();
  digitalWrite(rollEnable, LOW); // enable roll

  // Initialize pitch
  pitchServo1.attach(pitchPin1, 500, 2500);
  pitchServo2.attach(pitchPin2, 500, 2500);
  pitchSetSpeed(0);

  // Calibrate joystick
  for (int i = 0; i < 10; i++)
  {
    pitchCal += analogRead(pitchControlPin);
    rollCal += analogRead(rollControlPin);
    delay(10);
  }
  pitchCal /= 10;
  rollCal /= 10;
}

void loop()
{

  // // Control Gripper
  if (!digitalRead(gripClosePin))
  {
    closeGrip();
  }
  else if (!digitalRead(gripOpenPin))
  {
    openGrip();
  }
  else
  {
    stopGrip();
  }

  // Control Pitch
  int pitchControl = analogRead(pitchControlPin);
  if (abs(pitchControl - pitchCal) < potDeadBand)
  {
    pitchSetSpeed(0);
  }
  else
  {
    int pitchSpeed = map(pitchControl, 0, 1023, -100, 100);
    pitchSetSpeed(pitchSpeed);
  }

  // Control Roll
  int rollControl = analogRead(rollControlPin);
  if (abs(rollControl - rollCal) < potDeadBand)
  {
    zeroRoll();
    rollStepper.moveTo(0);
    rollStepper.stop();
  }
  else
  {
    int rollPos = map(rollControl, 0, 1023, -10 * microStepCoef, 10 * microStepCoef);
    rollStepper.move(rollPos);
    // int rollPos = map(rollControl, 0, 1023, -500 * microStepCoef, 500 * microStepCoef);
    // rollStepper.setSpeed(rollPos);
  }
  rollStepper.run();
  // rollStepper.runSpeed();
}

/*
 * -----------------------
 * ROLL CONTROL FUNCTIONS
 * -----------------------
 */
void zeroRoll()
{
  rollStepper.setCurrentPosition(0);
}

/*
 * ------------------------
 * PITCH CONTROL FUNCTIONS
 * ------------------------
 */
void pitchSetSpeed(int speed)
{
  if (speed == pitchCurrentSpeed)
  {
    return;
  }
  pitchCurrentSpeed = speed;
  // int speed2 = -1 * speed;

  int speedOut = map(speed, -100, 100, 0, 180);
  // int speedOut2 = map(speed2, -100, 100, 0, 180);

  pitchServo1.write(speedOut);
  // pitchServo2.write(speedOut2);
  pitchServo2.write(((speedOut - 90) * -1 + 90));
}

/*
 * ------------------------
 * GRIP CONTROL FUNCTIONS
 * ------------------------
 */
void openGrip()
{
  digitalWrite(gripPinA, LOW);
  digitalWrite(gripPinB, HIGH);
  if (debug)
  {
  }
}

void closeGrip()
{
  digitalWrite(gripPinA, HIGH);
  digitalWrite(gripPinB, LOW);
  if (debug)
  {
  }
}

void stopGrip()
{
  digitalWrite(gripPinA, LOW);
  digitalWrite(gripPinB, LOW);
  if (debug)
  {
  }
}
