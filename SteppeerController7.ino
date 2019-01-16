/*
 * Parts used
 * 
 * 1x Arduino
 * 1x MCP2515 CAN Transciever
 * 1x A4988 Stepper Driver
 * 1x NEMA17 stepper motor
 */



#include <AccelStepper.h>
#include <SPI.h>          //SPI is used to talk to the CAN Controller
#include <mcp_can.h>

//Stepper control with openpilot
//Set these variables for system config
double steerGear = 220; //steering gear diameter
double motorGear = 8; //motor gear diameter
//double angleError = 0; //degrees error needed to disengage
double microsteps = 4; //microstepping (1 = standard, 2 = half step, 4 = quarter step, 8  = eigth step)
int stepperSpeed = 2200;
int stepperAcceleration = 10000;

//set enable pin on A4988 driver
int enablePin = 5;

//init other variables
float gearRatio = 0;
float currentAngle = 0;
float desiredAngle = 0;
double lastDesiredAngle = 0;
double lastCurrentAngle = 0;
double angleDelta = 0;
double zeroAngle = 0;

long desiredStep = 0;
long lastDesiredStep = 0;
long currentStep = 0;

bool currentEngaged = false;
bool previousEngaged = false;
bool steerOverride = false;

double angleToStepFactor = 0;
double stepToAngleFactor = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned int canID;
unsigned char buf0 = 0;
unsigned char buf1 = 0;
unsigned char buf2 = 0;
unsigned char buf3 = 0;
unsigned char buf4 = 0;
unsigned char buf5 = 0;
unsigned char buf6 = 0;
unsigned char buf7 = 0;

float measure0 = 0;
float measure1 = 0;
float measure2 = 0;
float measure3 = 0;
float measure4 = 0;

int currentTime = 0;
int lastRequestTime = 0;
int lastReadTime = 0;
int lastGoToTime = 0;
int counter = 0;

float canCheck = 0;
unsigned char msg[2] = {0, 0};
int lastCANCheckTime = 0;


AccelStepper stepper(1, 8, 9); //(type(leave as 1 for a4988), step, direction)
MCP_CAN CANR(10); //CAN CS Pin



void steerCommandReceived() {
  //Serial.println("Update Received");

  //reads either 0 or 1
  buf0 = buf[0];
  currentEngaged = bitRead(buf0, 0);

  //combine 2 bufs to create big one
  buf1 = buf[1];
  buf2 = buf[2];
  measure1 = (buf1 << 8);
  measure2 = buf2;
  desiredAngle = measure1 + measure2;
  desiredAngle = desiredAngle / 100;

  lastRequestTime = millis();
  //Serial.println(desiredAngle);
  //Serial.println(currentEngaged);
  return;
}

void carAngleReceived() {

  buf0 = buf[0];
  buf1 = buf[1];
  buf4 = buf[4];
  measure0 = buf0;
  measure1 = buf1;
  measure4 = (buf4 >> 4);
  if (measure4 >= 8) {
    measure4 = -16 + measure4;
  }

  //if greater than 0, negative turn
  if (buf0 > 0) {
    currentAngle = -(384 - (measure1 * 1.5) - (measure4 / 10)); //15/16 converts to more accurate removed 15/16 because it seems more accurate to what OP is reading
  }
  else {
    currentAngle = (measure1 * 1.5) + (measure4 / 10); //15/16 converts to more accurate
  }


  //counter = counter +1;
  //Serial.println(counter);

  lastReadTime = millis();
  //Serial.print(measure1*1.5);
  //Serial.print("  ");
  Serial.println(currentAngle);
  //safetyCheck();
  return;
}

void safetyCheck() {

  return;
}

void disableLock() {
  //stepper.disableOutputs();
  //while (true){
  //delay(5000);
  //}
}

void updateDesiredStep() {

  
  angleDelta = desiredAngle - currentAngle;
  currentStep = stepper.currentPosition();

  //minus (-) because stepper is reversed direction
  desiredStep = currentStep - angleDelta * angleToStepFactor;  //currentStep + angleDelta * angleToStepFactor;
  //Serial.println(desiredStep);

//only update if reasonable change, need to determine a good number
  if (abs(lastDesiredStep - desiredStep) > 10) {

    lastDesiredStep = desiredStep;
    stepper.moveTo(desiredStep);
  }
  

  return;
}

void sendCANCheck() {

//CAN ID 500/0x1f4
  CANR.sendMsgBuf(0x1f4, 0, 2, msg);
  canCheck = canCheck + 1;
  msg[0] = canCheck;
  if (canCheck == 50)
  {
    canCheck = 0;
  }

  lastCANCheckTime = millis();

  //CANR.sendMsgBuf(0x512, 0, 2, msg); test
  //CANR.sendMsgBuf(0x513, 0, 2, msg);
  return;
}



void setup() {
  //set enable pin to disable immediately
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  Serial.begin(115200);
  
  //Initialize CAN
RETRYCANONE:
  if (CAN_OK == CANR.begin(CAN_500KBPS))     //setting CAN baud rate to 500Kbps
  {
    Serial.println("CAN BUS Receive Shield init ok!");
    delay(5);
  }

  else
  {
    Serial.println("CAN BUS Receive Shield init fail");
    goto RETRYCANONE;
  }


//calculate system parameters
  gearRatio = motorGear / steerGear;

  stepToAngleFactor = (gearRatio) * (360 / 200) * (1 / microsteps);
  angleToStepFactor = 1 / stepToAngleFactor;

  //set stepper speed and acceleration - tune these to each system
  stepper.setMaxSpeed(stepperSpeed); //steps per second
  stepper.setAcceleration(stepperAcceleration);

  //stepper.setCurrentPosition(0);

}



/*
 * loop() updates current steering angle from the car, and desired angle from Openpilot. 
 * Based on the difference it calculates how many steps to move to achieve the desired angle and requests to move there.
 * It continues to update at 200hz as both current angle and desired angle changes.
 * Also takes advantage of the enable pin on the A4988 to disable the stepper when not engaged.
 * 
 */
void loop() {

  //Check for a new CAN message and read if angle read or request
  if (CAN_MSGAVAIL == CANR.checkReceive())
  {
    CANR.readMsgBuf(&len, buf);
    canID = CANR.getCanId();

    //If the ones we care about...
    if (canID == 0x2e4) //angle request & engaged
    {
      steerCommandReceived();
    }
    else if (canID == 0x25) //steering angle
    {
      carAngleReceived();
    }
  }



  //update MoveTo every 5 milliseconds
  currentTime = millis();
  if ((currentTime - lastGoToTime) > 5 ) { //(desiredAngle != lastDesiredAngle) && 
    //Serial.println("run");
    lastDesiredAngle = desiredAngle;
    lastGoToTime = currentTime;
    updateDesiredStep();
  }



  //enable outputs when engaged
  if (currentEngaged == true)  { // && previousEngaged == false) {
    digitalWrite(enablePin, LOW);
    previousEngaged = currentEngaged;
  }

  //disable outputs when disengaged
  else if (currentEngaged == false)  { // && previousEngaged == true) {
    digitalWrite(enablePin, HIGH);
    previousEngaged = currentEngaged;
  }



  //Time check, disable if half second passes between CAN messages
  //currentTime = millis(); //comment out to use earlier time check
  if (((currentTime - lastRequestTime) > 250) || ((currentTime - lastReadTime) > 250)) {
    currentEngaged = false;
    digitalWrite(enablePin, HIGH);
  }

  if (
    ((currentTime - lastCANCheckTime) > 100))  {
    sendCANCheck();
  }

  //check if the stepper needs to move if engaged if engaged
  if (currentEngaged == true) {

    stepper.run();
  }

} //end loop



