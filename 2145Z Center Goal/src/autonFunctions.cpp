#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

// These are variables for sensor values
//double lineSensorValue1 = ballDetector1.value(pct);


double prevErrorInertial = 0;
double prevTarget = 0;
double prevTurn = 0;
double targetError = 0;
double ballIn = 0;

// This function initalizes the inertial sensor during pre auton
void inertialCalibration() {
  inertial_gyro.calibrate();
  while (inertial_gyro.isCalibrating()) {
    wait(10, msec);
  }
}
// This function resets all the values of the sensors
void reset() {
  tracker.resetRotation();
  tracker2.resetRotation();
}

void createBallCountTask(){
  task y = task(ballCycle);
}

void stopBallCountTask(){
  task::stop(ballCycle);
}
// This function allows our robot to move at a specfic speed until told to stop
void forwardTime(double speed, double t) {
  reset();
  FL.spin(directionType::rev, speed * 120, voltageUnits::mV);
  BL.spin(directionType::rev, speed * 120, voltageUnits::mV);
  FR.spin(directionType::rev, speed * 120, voltageUnits::mV);
  BR.spin(directionType::rev, speed * 120, voltageUnits::mV);
  
  wait(t, msec);

  FL.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  BL.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  FR.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  BR.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  targetError = inertial_gyro.rotation(degrees);
}
// This function allows our robot to move at a specfic speed until told to stop
void backwardTime(double speed, double t) {
  reset();
  FL.spin(directionType::fwd, speed * 120, voltageUnits::mV);
  BL.spin(directionType::fwd, speed * 120, voltageUnits::mV);
  FR.spin(directionType::fwd, speed * 120, voltageUnits::mV);
  BR.spin(directionType::fwd, speed * 120, voltageUnits::mV);
  wait(t, msec);
  FL.spin(directionType::fwd, 0 * 120, voltageUnits::mV);
  BL.spin(directionType::fwd, 0 * 120, voltageUnits::mV);
  FR.spin(directionType::fwd, 0 * 120, voltageUnits::mV);
  BR.spin(directionType::fwd, 0 * 120, voltageUnits::mV);
}

void forwardSpeed(double speed){
  FL.spin(reverse, speed, pct);
  BR.spin(reverse, speed, pct);
  FR.spin(reverse, speed, pct);
  BL.spin(reverse, speed, pct);
  targetError = inertial_gyro.rotation(degrees);
  reset();
  
}

void backwardSpeed(double speed){
  FL.spin(fwd, speed, pct);
  FR.spin(fwd, speed, pct);
  BL.spin(fwd, speed, pct);
  BR.spin(fwd, speed, pct);
  targetError = inertial_gyro.rotation(degrees);
  reset();
}

// This function allows our robot to set the speed of the rollers
void rollerSpeed(double speed) {
  RightRoller.spin(fwd, speed, pct);
  LeftRoller.spin(fwd, speed, pct);
}
// This function allows our robot to set the speed of the conveyor
void conveyorBottom(double speed) {
  Conveyor1.spin(fwd, speed, pct);
}
void conveyorTop(double speed) {
  Conveyor2.spin(fwd, speed, pct);
}
void index(double speed, double rollers){
  RightRoller.spin(fwd, rollers, pct);
  LeftRoller.spin(fwd, rollers, pct);
  Conveyor1.spin(fwd, speed, pct);
}

void spit(double speedRight, double speedLeft){
  RightRoller.spin(fwd, speedRight, pct);
  LeftRoller.spin(fwd, speedLeft, pct);  
}
// This function allows for all doubleaking subsystems to spin
void allSpin(double speed) {
  Conveyor1.spin(fwd, speed, pct);
  Conveyor2.spin(fwd, speed, pct);
  RightRoller.spin(fwd, speed, pct);
  LeftRoller.spin(fwd, speed, pct);
}
double counter = 0;
double counter2 = 0;
bool ball = false;
bool ball2 = false;

int ballCycle(){
  while(true){
    Brain.Screen.setCursor(7, 14);
    Brain.Screen.print(ballIn);
    printf("counter%f\n", counter);

  if(ballDetector2.value(pct) < 50 && ball2 == false){
    counter2++;
    ball2 = true;
  }
  else if(ballDetector2.value(pct) > 68){
    ball2 = false;
  }

  if(ballDetector3.value(pct) < 60 && ball == false){
      counter ++;
      ball = true;
  }
    else if(ballDetector3.value(pct) > 68){
      ball = false;
    }
  }
  return counter;
} 

//it descores 3

void goalScore(double Balls, double top){ 

  counter = 0;
  counter2 = 0;

  while(Balls > counter){
    printf("counter%f\n", counter);

    Conveyor1.spin(fwd, 100, pct);
    Conveyor2.spin(fwd, 100, pct);
    RightRoller.spin(fwd, 100, pct);
    LeftRoller.spin(fwd, 100, pct);

    if(counter2 == top){
      Conveyor1.spin(fwd, 0, pct);
      counter = Balls;
    }

    task::sleep(10);   
  }
    RightRoller.stop(brake);
    LeftRoller.stop(brake);
    Conveyor1.stop(brake);
    task::sleep(300);
    Conveyor2.stop(brake);
    counter = 0; 
    counter2 = 0;
}

void goalScore1(){

double counter = 0;

  while(counter == 0){
if(ballDetector3.value(pct) < 60){
  counter ++;
}
    Conveyor1.spin(fwd,100,pct);
    Conveyor2.spin(fwd, 50, pct);
    RightRoller.spin(fwd, 100, pct);
    LeftRoller.spin(fwd, 100, pct);     


  }
    RightRoller.stop(coast);
    LeftRoller.stop(coast);
    task::sleep(300);
    Conveyor1.stop(coast);
    Conveyor2.stop(coast);
    RightRoller.stop(coast);
    LeftRoller.stop(coast);
  
}




void ballCycleLine(){
  double counter = 0;
  while(counter < 1){
    Brain.Screen.setCursor(7, 14);
    Brain.Screen.print(ballIn);
    if(counter < 45){
      ballIn++;
      counter++;
      task::sleep(100);
      
    }
  }
  counter = 0;
}


void goalScoreLine(double speed, double balls){ 
  while(ballIn < balls){
    Conveyor1.spin(fwd, speed, pct);
    Conveyor2.spin(fwd, speed, pct);
    RightRoller.spin(reverse, 100, pct);
    LeftRoller.spin(reverse, 100, pct);

    ballCycleLine();
  }
  ballIn = 0;
  Conveyor1.stop(hold);
  Conveyor2.stop(hold);
  RightRoller.stop(coast);
  LeftRoller.stop(coast);
}

// Sets the conveyor to coast after being set to hold
void setCoast() {
  FL.stop(coast);
  FR.stop(coast);
  BL.stop(coast);
  BR.stop(coast);
  Conveyor1.stop(coast);
  Conveyor2.stop(coast);
}
void setHold() {
  FL.stop(hold);
  FR.stop(hold);
  BL.stop(hold);
  BR.stop(hold);
}
void flipOut() {
  // conveyorSpeed(-100);
  rollerSpeed(100);
  task::sleep(200);
  rollerSpeed(0);
}

void printTracker() {
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("tracker:");
  Brain.Screen.setCursor(5, 14);
  Brain.Screen.print(tracker.rotation(deg));
}


void forwardPID(double target, double headingVal, double counterThresh, double accuracy) {
  // Constants
  double kP = 0.13;
  double kPAngle = 4;//4
  double kI = 0.09;//.09
  double kD = 0.3;//.3
  double kDAngle = 6;//6

  double errorRight = 0;
  double errorLeft = 0;
  double errorInertial = 0;
  double totalErrorRight = 0;
  double totalErrorLeft = 0;
  double prevErrorRight = 0;
  double prevErrorLeft = 0;
  double derivativeRight = 0;
  double derivativeLeft = 0;
  double counterLeft = 0;
  double counterRight = 0;
  double derivativeInertial = 0;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheelRight = fabs(tracker.rotation(deg));
  double trackingWheelLeft = fabs(tracker2.rotation(deg));
  errorInertial = prevErrorInertial;


  while (counterLeft < counterThresh || counterRight < counterThresh) {//counterLeft < counterThresh || counterRight < counterThresh
    //Update sensor values
    
    trackingWheelRight = fabs(tracker.rotation(deg));
    trackingWheelLeft = fabs(tracker2.rotation(deg));
    errorInertial = headingVal - inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 7;

    // Proportional
    errorLeft = target - trackingWheelRight;
    errorRight = target - trackingWheelLeft;

    // Integral
    totalErrorRight += errorRight;
    totalErrorLeft += errorLeft;


    // introduces I term when needed

    if (errorRight > 150) {
      totalErrorRight = 0;
    }

    if (fabs(errorRight) < 150) {
      totalErrorRight = 20;
    }

    // Derivative
    derivativeRight = errorRight - prevErrorRight;

    if (errorLeft > 150) {
      totalErrorLeft = 0;
    }

    if (fabs(errorLeft) < 150) {
      totalErrorLeft = 20;
    }

    // Derivative
    derivativeLeft = errorLeft - prevErrorLeft;

    // Derivative Inertial
    derivativeInertial = errorInertial - prevErrorInertial;

   

    // Find the speed of chassis based of the sum of the constants
    double motorPowerRight = (kP * errorRight) + (kI * totalErrorRight) + (kD * derivativeRight);
    double motorPowerLeft = (kP * errorLeft) + (kI * totalErrorLeft) + (kD * derivativeLeft);
    double heading = (kPAngle * errorInertial) + (kDAngle * derivativeInertial);


    // If the motorPower is larger then the limit, the motor power will equal

    // the limit

    if (motorPowerLeft > 90) {
      motorPowerLeft = 90;
    }

    if (motorPowerRight > 90) {
      motorPowerRight = 90;
    }

    if (limit < motorPowerRight) {
      motorPowerRight = limit;
    }
 
    if (fabs(motorPowerRight) < 10) {
      motorPowerRight = 10;
    }

    if (limit < motorPowerLeft) {
      motorPowerLeft = limit;
    }
 
    if (fabs(motorPowerLeft) < 10) {
      motorPowerLeft = 10;
    }

    // Sets the speed of the drive
    FL.spin(directionType::rev, 110 * (motorPowerLeft + heading),
            voltageUnits::mV);
    BL.spin(directionType::rev, 110 * (motorPowerLeft + heading),
            voltageUnits::mV);
    FR.spin(directionType::rev, 110 * (motorPowerRight - heading),
            voltageUnits::mV);
    BR.spin(directionType::rev, 110 * (motorPowerRight - heading),
            voltageUnits::mV);

    prevErrorRight = errorRight;
    prevErrorLeft = errorLeft;
    prevErrorInertial = errorInertial;
    if (fabs(errorRight) <= accuracy) {
      counterRight += 1;
    }
    if (fabs(errorRight) >= accuracy) {
      counterRight = 0;
    }
    if (fabs(errorLeft) <= accuracy) {
      counterLeft += 1;
    }
    if (fabs(errorLeft) >= accuracy) {
      counterLeft = 0;
    }

    

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  //printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
  setCoast();
}
// This function moves the robot backwards using a PID controller
void backwardPID(double target, double headingVal, double counterThresh, double accuracy){
  // Constants
  double kP = 0.13;
  double kPAngle = 4;//4
  double kI = 0.09;//.09
  double kD = 0.3;//.3
  double kDAngle = 6;//6

  double errorRight = 0;
  double errorLeft = 0;
  double errorInertial = 0;
  double totalErrorRight = 0;
  double totalErrorLeft = 0;
  double prevErrorRight = 0;
  double prevErrorLeft = 0;
  double derivativeRight = 0;
  double derivativeLeft = 0;
  double counterLeft = 0;
  double counterRight = 0;
  double counterLeft2 = 0;
  double counterRight2 = 0;
  double derivativeInertial = 0;
  double limit = 0;
  double accuracy2 = 500;
  double counterThresh2 = 500;
  bool run = true;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheelRight = fabs(tracker.rotation(deg));
  double trackingWheelLeft = fabs(tracker2.rotation(deg));
  errorInertial = prevErrorInertial;


  while (run) {
    //Update sensor values
    
    trackingWheelRight = fabs(tracker.rotation(deg));
    trackingWheelLeft = fabs(tracker2.rotation(deg));
    errorInertial = headingVal - inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 7;

    // Proportional
    errorLeft = target - trackingWheelRight;
    errorRight = target - trackingWheelLeft;

    // Integral
    totalErrorRight += errorRight;
    totalErrorLeft += errorLeft;


    // introduces I term when needed

    if (errorRight > 150) {
      totalErrorRight = 0;
    }

    if (fabs(errorRight) < 150) {
      totalErrorRight = 20;
    }

    // Derivative
    derivativeRight = errorRight - prevErrorRight;

    if (errorLeft > 150) {
      totalErrorLeft = 0;
    }

    if (fabs(errorLeft) < 150) {
      totalErrorLeft = 20;
    }

    // Derivative
    derivativeLeft = errorLeft - prevErrorLeft;

    // Derivative Inertial
    derivativeInertial = errorInertial - prevErrorInertial;

   

    // Find the speed of chassis based of the sum of the constants
    double motorPowerRight = (kP * errorRight) + (kI * totalErrorRight) + (kD * derivativeRight);
    double motorPowerLeft = (kP * errorLeft) + (kI * totalErrorLeft) + (kD * derivativeLeft);
    double heading = (kPAngle * errorInertial) + (kDAngle * derivativeInertial);


    // If the motorPower is larger then the limit, the motor power will equal

    // the limit

    if (motorPowerLeft > 90) {
      motorPowerLeft = 90;
    }

    if (motorPowerRight > 90) {
      motorPowerRight = 90;
    }

    if (limit < motorPowerRight) {
      motorPowerRight = limit;
    }
 
    if (fabs(motorPowerRight) < 10) {
      motorPowerRight = 10;
    }

    if (limit < motorPowerLeft) {
      motorPowerLeft = limit;
    }
 
    if (fabs(motorPowerLeft) < 10) {
      motorPowerLeft = 10;
    }

    // Sets the speed of the drive
    FL.spin(directionType::fwd, 110 * (motorPowerLeft - heading),
            voltageUnits::mV);
    BL.spin(directionType::fwd, 110 * (motorPowerLeft - heading),
            voltageUnits::mV);
    FR.spin(directionType::fwd, 110 * (motorPowerRight + heading),
            voltageUnits::mV);
    BR.spin(directionType::fwd, 110 * (motorPowerRight + heading),
            voltageUnits::mV);

    prevErrorRight = errorRight;
    prevErrorLeft = errorLeft;
    prevErrorInertial = errorInertial;

    if (fabs(errorRight) <= accuracy) {
      counterRight += 1;
    }
    if (fabs(errorRight) >= accuracy) {
      counterRight = 0;
    }
    if (fabs(errorLeft) <= accuracy) {
      counterLeft += 1;
    }
    if (fabs(errorLeft) >= accuracy) {
      counterLeft = 0;
    }

    if (fabs(errorRight) <= accuracy2) {
      counterRight2 += 1;
    }
    if (fabs(errorRight) >= accuracy2) {
      counterRight2 = 0;
    }
    if (fabs(errorLeft) <= accuracy2) {
      counterLeft2 += 1;
    }
    if (fabs(errorLeft) >= accuracy2) {
      counterLeft2 = 0;
    }

    if (counterLeft2 < counterThresh2 && counterRight2 < counterThresh2){
      run = false;
    }

    if (counterLeft < counterThresh && counterRight < counterThresh) {
      run = false;
    }

    

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  //printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
  setCoast();
}

// This function turns the robot right using a PID controller
// 45-90 5, 140, 8
void rightPID(double target, double counterThresh, double accuracy, double maxSpeed, double kP, double kI, double kD) {
  // Constants
  //double kP = 1.34;//1.34
  //double kI = 0.04;//.04
  //double kD = 3;//3

  double counter = 0;
  double error = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double inetVal = inertial_gyro.rotation(degrees);
  // targetError = prevTurn - prevTarget;
  // target = target - targetError;

  //while (counter < counterThresh) {
  while (counter < counterThresh) {

    // Update sensor values
    // target = prevTarget;

    printf("velocity %f/n", FR.velocity(pct));
    inetVal = inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 15;

    // Proportional
    error = target - inetVal;

    // Integral
    totalError += error;

    if (error > 3) {
      totalError = 0;
    }

    if (fabs(error) < 3) {
      totalError = 20;
    }

    if (fabs(error) > 10) {
      totalError = 0;
    }

    if (fabs(error) < 10) {
      totalError += error;
    }

    // Derivative
    derivative = error - prevError;

    double motorPower = (kP * error) + (kI * totalError) + (kD * derivative);

    // If the motorPower is larger then the limit, the motor power will equal
    // the limit
    if (limit < motorPower) {
      motorPower = limit;
    }

    if (motorPower > maxSpeed) {
      motorPower = maxSpeed;
    }

    // Sets the speed of the drive
    FL.spin(directionType::rev, motorPower * 120, voltageUnits::mV);
    BL.spin(directionType::rev, motorPower * 120, voltageUnits::mV);
    FR.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);
    BR.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);

    prevError = error;
    printInet();
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Error:");
    Brain.Screen.setCursor(2, 14);
    Brain.Screen.print(error);
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }
     printf("errorInertial %f\n", inertial_gyro.rotation(deg));

    task::sleep(10);
  }

  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control slowly
  // inetVal = prevTurn;
    printf("errorInertial %f\n", inertial_gyro.rotation(deg));

  reset();
  setHold();
  setCoast();
}

void leftPID(double target, double counterThresh, double accuracy, double maxSpeed, double kP, double kI, double kD) {
  // Constants
  //double kP = 1.34;//1.34
  //double kI = 0.04;//.04
  //double kD = 3;//3

  double counter = 0;
  double error = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double inetVal = inertial_gyro.rotation(degrees);
  // targetError = prevTurn - prevTarget;
  // target = target - targetError;

  while (counter < counterThresh) {

    // Update sensor values
    // target = prevTarget;
    printf("velocity %f/n", FR.velocity(pct));
    inetVal = fabs(inertial_gyro.rotation(degrees));

    // Update the limit
    limit += 15;

    // Proportional
    error = target - inetVal;

    // Integral
    totalError += error;

    if (fabs(error) > 15) {
      totalError = 0;
    }

    if (fabs(error) < 15) {
      totalError += error;
    }

    // Derivative
    derivative = error - prevError;

    double motorPower = (kP * error) + (kI * totalError) + (kD * derivative);

    // If the motorPower is larger then the limit, the motor power will equal
    // the limit
    if (limit < motorPower) {
      motorPower = limit;
    }

    if (motorPower > maxSpeed) {
      motorPower = maxSpeed;
    }

    // Sets the speed of the drive
    FL.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);
    BL.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);
    FR.spin(directionType::rev, motorPower * 120, voltageUnits::mV);
    BR.spin(directionType::rev, motorPower * 120, voltageUnits::mV);

    prevError = error;
    printInet();
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Error:");
    Brain.Screen.setCursor(2, 14);
    Brain.Screen.print(error);
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }

    task::sleep(10);
  }

  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control slowly
  // inetVal = prevTurn;
  printf("%f\n", inertial_gyro.rotation(degrees));
  reset();
  setHold();
  setCoast();
}

// This function moves the robot forwards using a PID controller
void forwardPIDLight(double target, double headingVal, double counterThresh, double accuracy) {
  // Constants
  double kP = 0.13;
  double kPAngle = 5;
  double kI = 0.09;
  double kD = 0.3;
  double kDAngle = 2.5;

  double error = 0;
  double errorInertial = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double counter = 0;
  double derivativeInertial = 0;
  double limit = 0;
  double counterBump = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheel = ((fabs(tracker2.rotation(deg))+(fabs(tracker2.rotation(deg))))/2);
  errorInertial = prevErrorInertial;


  while (counter < counterThresh && counterBump == 0) {
    //Update sensor values
    
    trackingWheel = ((fabs(tracker2.rotation(deg))+(fabs(tracker2.rotation(deg))))/2);
    errorInertial = headingVal - inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 7;

    if(bumper1.value() > 0){
      counterBump++;
    }

    // Proportional
    error = target - trackingWheel;

    // Integral
    totalError += error;

    // introduces I term when needed
    if (error > 150) {
      totalError = 0;
    }

    if (fabs(error) < 150) {
      totalError = 20;
    }

    // Derivative
    derivative = error - prevError;

    // Derivative Inertial
    derivativeInertial = errorInertial - prevErrorInertial;

   

    // Find the speed of chassis based of the sum of the constants
    double motorPower = (kP * error) + (kI * totalError) + (kD * derivative);
    double heading = (kPAngle * errorInertial) + (kDAngle * derivativeInertial);


    // If the motorPower is larger then the limit, the motor power will equal

    // the limit
    if (limit < motorPower) {
      motorPower = limit;
    }
 
    if (fabs(motorPower) < 10) {
      motorPower = 10;
    }

   /* if (motorPower > 90) {
      motorPower = 90;
    }
*/
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Tracker:");
    Brain.Screen.setCursor(4, 14);
    Brain.Screen.print(trackingWheel);

    // Sets the speed of the drive
    FL.spin(directionType::rev, 110 * (motorPower + (motorPower / 90 * heading)),
            voltageUnits::mV);
    BL.spin(directionType::rev, 110 * (motorPower + (motorPower / 90 * heading)),
            voltageUnits::mV);
    FR.spin(directionType::rev, 110 * (motorPower - (motorPower / 90 * heading)),
            voltageUnits::mV);
    BR.spin(directionType::rev, 110 * (motorPower - (motorPower / 90 * heading)),
            voltageUnits::mV);

    prevError = error;
    prevErrorInertial = errorInertial;
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }

    

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  //printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
  setCoast();
}

// This function moves the robot backwards using a PID controller

void leftTurn(double target, double counterThresh, double accuracy, double maxSpeed, double Speed) {
  // Constants
  //double kP = 1.34;//1.34
  //double kI = 0.04;//.04
  //double kD = 3;//3

  double counter = 0;
  double error = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double inetVal = inertial_gyro.rotation(degrees);
  // targetError = prevTurn - prevTarget;
  // target = target - targetError;

  while (target > inetVal) {

    // Update sensor values
    // target = prevTarget;
    printf("velocity %f/n", FR.velocity(pct));
    inetVal = fabs(inertial_gyro.rotation(degrees));

    // Update the limit
    limit += 15;

    // Proportional
    error = target - inetVal;

    // Integral
    totalError += error;

    if (fabs(error) > 15) {
      totalError = 0;
    }

    if (fabs(error) < 15) {
      totalError += error;
    }

    // Derivative
    derivative = error - prevError;

    double motorPower = Speed;

    // If the motorPower is larger then the limit, the motor power will equal
    // the limit
    if (limit < motorPower) {
      motorPower = limit;
    }

    if (motorPower > maxSpeed) {
      motorPower = maxSpeed;
    }

    // Sets the speed of the drive
    FL.spin(directionType::fwd, Speed * 120, voltageUnits::mV);
    BL.spin(directionType::fwd, Speed * 120, voltageUnits::mV);
    FR.spin(directionType::rev, Speed * 120, voltageUnits::mV);
    BR.spin(directionType::rev, Speed * 120, voltageUnits::mV);

    prevError = error;
    printInet();
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Error:");
    Brain.Screen.setCursor(2, 14);
    Brain.Screen.print(error);
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }

    task::sleep(10);
  }

  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control slowly
  // inetVal = prevTurn;
  printf("%f\n", inertial_gyro.rotation(degrees));
  reset();
  setHold();
  setCoast();
}

// This function moves the robot forwards using a PID controller
void forwardPIDcurve(double target, double headingVal, double counterThresh, double accuracy) {
  // Constants
  double kP = 0.13;
  double kPAngle = 4;
  double kI = 0.09;
  double kD = 0.3;
  double kDAngle = 6;

  double error = 0;
  double errorInertial = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double counter = 0;
  double derivativeInertial = 0;
  double limit = 0;
  double counterBump = 0;
  
  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheel = ((fabs(tracker2.rotation(deg))+(fabs(tracker2.rotation(deg))))/2);
  errorInertial = prevErrorInertial;


  while (counter < counterThresh && counterBump == 0) { {
    //Update sensor values
    
    trackingWheel = ((fabs(tracker2.rotation(deg))+(fabs(tracker2.rotation(deg))))/2);
    errorInertial = headingVal - inertial_gyro.rotation(degrees);

    if(bumper1.value() > 0){
      counterBump++;
    }

    // Update the limit
    limit += 7;

    // Proportional
    error = target - trackingWheel;

    // Integral
    totalError += error;

    // introduces I term when needed
    if (error > 150) {
      totalError = 0;
    }

    if (fabs(error) < 150) {
      totalError = 20;
    }

    // Derivative
    derivative = error - prevError;

    // Derivative Inertial
    derivativeInertial = errorInertial - prevErrorInertial;

   

    // Find the speed of chassis based of the sum of the constants
    double motorPower = (kP * error) + (kI * totalError) + (kD * derivative);
    double heading = (kPAngle * errorInertial) + (kDAngle * derivativeInertial);


    // If the motorPower is larger then the limit, the motor power will equal

    // the limit
    if (limit < motorPower) {
      motorPower = limit;
    }
 
    if (fabs(motorPower) < 10) {
      motorPower = 10;
    }

   /* if (motorPower > 90) {
      motorPower = 90;
    }
*/
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Tracker:");
    Brain.Screen.setCursor(4, 14);
    Brain.Screen.print(trackingWheel);

    // Sets the speed of the drive
    FL.spin(directionType::rev, 110 * ((motorPower + (motorPower / 90))/4),
            voltageUnits::mV);
    BL.spin(directionType::rev, 110 * ((motorPower + (motorPower / 90))/4),
            voltageUnits::mV);
    FR.spin(directionType::rev, 110 * (motorPower - (motorPower / 90)),
            voltageUnits::mV);
    BR.spin(directionType::rev, 110 * (motorPower - (motorPower / 90)),
            voltageUnits::mV);

    prevError = error;
    prevErrorInertial = errorInertial;
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }

    

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  //printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
  setCoast();
}
}
