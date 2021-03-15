#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

// These are variables for sensor values
double lineSensorValue1 = ballDetector1.value(pct);
double lineSensorValue2 = ballDetector2.value(pct);
double lineSensorValue3 = ballDetector3.value(pct);

double prevErrorInertial = 0;
double prevTarget = 0;
double prevTurn = 0;
double targetError = 0;
double ballIn = 0;

// This function initalizes the inertial sensor during pre auton
void inertialCalibration() {
  inertial_gyro.calibrate();
  while (inertial_gyro.isCalibrating()) {
    wait(2000, msec);
  }
}
// This function resets all the values of the sensors
void reset() {
  tracker.resetRotation();
  inertial_gyro.resetRotation();
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
  RightRoller.spin(reverse, speed, pct);
  LeftRoller.spin(reverse, speed, pct);
}
// This function allows our robot to set the speed of the conveyor
void conveyorSpeed(double speed) {
  Conveyor1.spin(fwd, speed, pct);
  Conveyor2.spin(fwd, speed, pct);
}
// This function allows for all doubleaking subsystems to spin
void allSpin(double speed) {
  Conveyor1.spin(fwd, speed, pct);
  Conveyor2.spin(fwd, speed, pct);
  RightRoller.spin(reverse, speed, pct);
  LeftRoller.spin(reverse, speed, pct);
}

void ballCycle(){
  double counter = 0;
  while(counter < 1){
    Brain.Screen.setCursor(7, 14);
    Brain.Screen.print(ballIn);
    if(limit1.value() !=0){
      ballIn++;
      counter++;
      task::sleep(100);
      
    }
  }
  counter = 0;
  
} 

void goalScore(double speed, double balls){ 
  while(ballIn < balls){
    Conveyor1.spin(fwd, speed, pct);
    Conveyor2.spin(fwd, speed, pct);
    RightRoller.spin(reverse, 100, pct);
    LeftRoller.spin(reverse, 100, pct);

    ballCycle();
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


// This function moves the robot forwards using a PID controller
void forwardPID(double target) {
  // Constants
  double kP = 0.15;
  double kPAngle = 6;
  double kI = 0;
  double kD = 0.25;
  double kDAngle = 0;

  double error = 0;
  double errorInertial = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double derivativeInertial = 0;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheel = fabs(tracker.rotation(deg));


  while (target > trackingWheel) {
    //printf("error%f\n", targetError);
    // Update sensor values
    trackingWheel = fabs(tracker.rotation(deg));

    // Update the limit
    limit += 3;

    // Proportional
    error = target - trackingWheel;

    // doubleegral
    totalError += error;

    // doubleroduces I term when needed
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

    if(errorInertial < 0.7){
      kPAngle = 0;
    }
    else{
      kPAngle = 5;
    }

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

    if (motorPower > 90) {
      motorPower = 90;
    }

    // Sets the speed of the drive
    FL.spin(directionType::rev, 110 * (motorPower - motorPower / 90 * heading),
            voltageUnits::mV);
    BL.spin(directionType::rev, 110 * (motorPower - motorPower / 90 * heading),
            voltageUnits::mV);
    FR.spin(directionType::rev, 110 * (motorPower + motorPower / 90 * heading),
            voltageUnits::mV);
    BR.spin(directionType::rev, 110 * (motorPower + motorPower / 90 * heading),
            voltageUnits::mV);

    prevError = error;

      printf("heading%f\n", heading);
    
    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertadoubley and
  // then set the coast for drive control
  printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
}

// This function moves the robot backwards using a PID controller
void backwardPID(double target, double previousHeading) {
  // Constants
  double kP = 0.15;
  double kPAngle = 6;
  double kI = 0;
  double kD = 0.25;
  double kDAngle = 0;

  double error = 0;
  double errorInertial = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double derivativeInertial = 0;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheel = fabs(tracker.rotation(deg));


  while (target > trackingWheel) {
    //printf("error%f\n", targetError);
    // Update sensor values
    trackingWheel = fabs(tracker.rotation(deg));

    // Update the limit
    limit += 3;

    // Proportional
    error = target - trackingWheel;

    // doubleegral
    totalError += error;

    // doubleroduces I term when needed
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

    if(errorInertial < 0.5){
      kPAngle = 0;
    }
    else{
      kPAngle = 5;
    }

    // Find the speed of chassis based of the sum of the constants
    double motorPower = (kP * error) + (kI * totalError) + (kD * derivative);
    double heading = previousHeading;
    

    // If the motorPower is larger then the limit, the motor power will equal
    // the limit
    if (limit < motorPower) {
      motorPower = limit;
    }

    if (fabs(motorPower) < 10) {
      motorPower = 10;
    }

    if (motorPower > 90) {
      motorPower = 90;
    }

    // Sets the speed of the drive
    FL.spin(directionType::fwd, 110 * (motorPower + motorPower / 90 * heading),
            voltageUnits::mV);
    BL.spin(directionType::fwd, 110 * (motorPower + motorPower / 90 * heading),
            voltageUnits::mV);
    FR.spin(directionType::fwd, 110 * (motorPower - motorPower / 90 * heading),
            voltageUnits::mV);
    BR.spin(directionType::fwd, 110 * (motorPower - motorPower / 90 * heading),
            voltageUnits::mV);

    prevError = error;
      printf("heading%f\n", heading);
    
    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertadoubley and
  // then set the coast for drive control
  printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
}

// This function turns the robot right using a PID controller
// 45-90 5, 140, 8
void rightPID(double target, double counterThresh, double accuracy) {
  // Constants
  double kP = 1.4;
  double kI = 0.06;
  double kD = 5;

  double counter = 0;
  double error = 0;
  double totalError = 0;
  double prevError = 0;
  double derivative;
  double limit = 0;
  double inetVal = inertial_gyro.rotation(degrees);
   targetError = prevTarget - prevTurn;
   target = target + targetError;



  while (counter < counterThresh) {

    // Update sensor values
    inetVal = inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 5;

    // Proportional
    error = target - inetVal;

    // doubleegral
    totalError += error;

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

    if (motorPower > 75) {
      motorPower = 75;
    }

    // Sets the speed of the drive
    FL.spin(directionType::rev, motorPower * 120, voltageUnits::mV);
    BL.spin(directionType::rev, motorPower * 120, voltageUnits::mV);
    FR.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);
    BR.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);

    prevError = error;
   
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }

    task::sleep(10);
  }

  // When the loop ends, the motors are set to brake for less uncertadoubley and
  // then set the coast for drive control slowly
  prevTarget = target;
  printf("targetError%f\n", targetError);
  reset();
  setHold();
  //setCoast();
}

void leftPID(double target, double counterThresh, double accuracy) {
  // Constants
  double kP = 1.4;
  double kI = 0.04;
  double kD = 5;

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
if(prevTurn > 0){
  targetError = prevTarget - prevTurn;
  target = target - targetError;
}
else{
  targetError = prevTarget - fabs(prevTurn);
  target = target + targetError;

}

  while (counter < counterThresh) {

    // Update sensor values
    // target = prevTarget;
    inetVal = fabs(inertial_gyro.rotation(degrees));

    // Update the limit
    limit += 5;

    // Proportional
    error = target - inetVal;

    // doubleegral
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

    // Sets the speed of the drive
    FL.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);
    BL.spin(directionType::fwd, motorPower * 120, voltageUnits::mV);
    FR.spin(directionType::rev, motorPower * 120, voltageUnits::mV);
    BR.spin(directionType::rev, motorPower * 120, voltageUnits::mV);

    prevError = error;
    
    if (fabs(error) <= accuracy) {
      counter += 1;
    }
    if (fabs(error) >= accuracy) {
      counter = 0;
    }

    task::sleep(10);
  }

  // When the loop ends, the motors are set to brake for less uncertadoubley and
  // then set the coast for drive control slowly
  prevTarget = target;
  printf("targetError%f\n", targetError);
  reset();
  setHold();
}

