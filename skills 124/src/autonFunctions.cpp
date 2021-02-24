#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

// These are variables for sensor values
int lineSensorValue1 = ballDetector1.value(pct);
int lineSensorValue2 = ballDetector2.value(pct);
int lineSensorValue3 = ballDetector3.value(pct);
int opticalHue1 = optical1.hue();

// This function initalizes the inertial sensor during pre auton
void inertialCalibration() {
  inertial_gyro.calibrate();
  while (inertial_gyro.isCalibrating()) {
    wait(2000, msec);
  }
}
// This function resets all the values of the sensors
void reset() {
  BL.resetRotation();
  BR.resetRotation();
  FL.resetRotation();
  FR.resetRotation();
  tracker.resetRotation();
  inertial_gyro.resetRotation();
}
// This function allows our robot to move forwards for a certain amount of
// degrees and for a set speed
void moveForward(int distance, int speed) {
  double wheelDiameterIN = 3.25;
  double travelTargetCM =
      distance; // this is the distance it goes which is set as a variable
  double circumfrence = wheelDiameterIN * 3.141592;
  double degreesToRotate = ((360 * travelTargetCM) / circumfrence) * sin(45);

  BL.setVelocity(speed, vex::velocityUnits::pct);
  BR.setVelocity(speed, vex::velocityUnits::pct);
  FL.setVelocity(speed, vex::velocityUnits::pct);
  FR.setVelocity(speed, vex::velocityUnits::pct);

  BL.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  BR.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  FL.rotateFor(-degreesToRotate, vex::rotationUnits::deg, false);
  FR.rotateFor(-degreesToRotate, vex::rotationUnits::deg, true);
}
// This function allows our robot to move backwards for a certain amount of
// degrees and for a set speed
void backward(int distance, int speed) {
  while (true) {
    FL.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
    FR.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
    BR.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
    BL.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
  }
}
// This function allows our robot to move at a specfic speed until told to stop
void forwardTime(int speed, double t) {
  FL.spin(directionType::rev, speed * 120, voltageUnits::mV);
  BL.spin(directionType::rev, speed * 120, voltageUnits::mV);
  FR.spin(directionType::rev, speed * 120, voltageUnits::mV);
  BR.spin(directionType::rev, speed * 120, voltageUnits::mV);
  
  wait(t, msec);

  FL.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  BL.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  FR.spin(directionType::rev, 0 * 120, voltageUnits::mV);
  BR.spin(directionType::rev, 0 * 120, voltageUnits::mV);
}
// This function allows our robot to move at a specfic speed until told to stop
void backwardTime(int speed, double t) {
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

void forwardSpeed(int speed){
  FL.spin(reverse, speed, pct);
  FR.spin(reverse, speed, pct);
  BL.spin(reverse, speed, pct);
  BR.spin(reverse, speed, pct);
}

void backwardSpeed(int speed){
  FL.spin(fwd, speed, pct);
  FR.spin(fwd, speed, pct);
  BL.spin(fwd, speed, pct);
  BR.spin(fwd, speed, pct);
}
// This function allows our robot to intake balls for a certain amount of
// degrees and for a set speed
void intakeRollers(int distance, int speed) {
  RightRoller.rotateFor(distance, rotationUnits::deg, speed,
                        velocityUnits::pct);
  LeftRoller.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
}
// This function allows our robot to outtake balls for a certain amount of
// degrees and for a set speed
void outtakeRollers(int distance, int speed) {
  RightRoller.rotateFor(-distance, rotationUnits::deg, speed,
                        velocityUnits::pct);
  LeftRoller.rotateFor(-distance, rotationUnits::deg, speed,
                       velocityUnits::pct);
}
// This function allows our robot to intake and score balls for a certain amount
// of degrees and for a set speed
void intakeConveyor(int distance, int speed) {
  Conveyor1.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
  Conveyor2.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
}
// This function allows our robot to downttake and get rid of balls for a
// certain amount of degrees and for a set speed
void outtakeConveyor(int distance, int speed) {
  Conveyor1.rotateFor(-distance, rotationUnits::deg, speed, velocityUnits::pct);
  Conveyor2.rotateFor(-distance, rotationUnits::deg, speed, velocityUnits::pct);
}
// This function allows our robot to set the speed of the rollers
void rollerSpeed(int speed) {
  RightRoller.spin(reverse, speed, pct);
  LeftRoller.spin(reverse, speed, pct);
}
// This function allows our robot to set the speed of the conveyor
void conveyorSpeed(int speed) {
  Conveyor1.spin(fwd, speed, pct);
  Conveyor2.spin(fwd, speed, pct);
}
// This function allows for all intaking subsystems to spin
void allSpin(int speed) {
  Conveyor1.spin(fwd, speed, pct);
  Conveyor2.spin(fwd, speed, pct);
  RightRoller.spin(reverse, speed, pct);
  LeftRoller.spin(reverse, speed, pct);
}

// This function stops red balls at the top of the conveyor for optimal priming
// position
void ballStopRed() {
  if (lineSensorValue2 <= 60 || lineSensorValue3 <= 60 || opticalHue1 < 40) {
    Conveyor1.stop(hold);
    Conveyor2.stop(hold);
    RightRoller.stop(coast);
    LeftRoller.stop(coast);
  }
}
// This function stops blue balls at the top of the conveyor for optimal priming
// position
void goalScore(int speed) {
  int counter = 0;
  // int counter2 = 0;

  Conveyor1.spin(fwd, speed, pct);
  Conveyor2.spin(fwd, speed, pct);
  RightRoller.spin(reverse, 100, pct);
  LeftRoller.spin(reverse, 100, pct);

  while (counter < 370) {
    // refresh sensor values
    lineSensorValue2 = ballDetector2.value(pct);
    lineSensorValue3 = ballDetector3.value(pct);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Counter:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(counter);

    printLineValue2();

    if (lineSensorValue2 <= 60 || lineSensorValue3 <= 60) {
      counter += 1;
    }
  }
  Conveyor1.stop(hold);
  Conveyor2.stop(hold);
  RightRoller.stop(coast);
  LeftRoller.stop(coast);
}

// This function stops all balls at the top of the conveyor for optimal priming
// position
void ballStopAll() {
  if (lineSensorValue2 <= 60 || lineSensorValue3 <= 60 || opticalHue1 < 40 ||
      opticalHue1 > 100) {
    Conveyor1.stop(hold);
    Conveyor2.stop(hold);
  }
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
void printAlexi() {
  while (true) {
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print(inertial_gyro.rotation(degrees));
  }
}

void printSexy() {
  while (true) {
    printf("peen %f\n", inertial_gyro.rotation(degrees));
  }
}

// This function moves the robot forwards using a PID controller
void forwardPID(int target) {
  // Constants
  double kP = 0.15;
  double kPAngle = 3;
  double kI = 0;
  double kD = 0.25;
  double kDAngle = 0;

  double error = 0;
  double errorInertial = 0;
  double totalError = 0;
  double prevError = 0;
  double prevErrorInertial = 0;
  double derivative;
  double derivativeInertial = 0;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheel = fabs(tracker.rotation(deg));
  errorInertial = inertial_gyro.rotation(degrees);

  while (target > trackingWheel) {
    // Update sensor values
    trackingWheel = fabs(tracker.rotation(deg));
    errorInertial = inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 3;

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

    if (motorPower > 90) {
      motorPower = 90;
    }

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Tracker:");
    Brain.Screen.setCursor(4, 14);
    Brain.Screen.print(trackingWheel);

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
    prevErrorInertial = errorInertial;

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  printf("tracker%f\n", tracker.rotation(degrees));
  reset();
  setHold();
  setCoast();
}

// This function moves the robot backwards using a PID controller
void backwardPID(int target) {
  // Constants
  double kP = 0.15;
  double kPAngle = 3;
  double kI = 0;
  double kD = 0.25;
  double kDAngle = 0;

  double error = 0;
  double errorInertial = 0;
  double totalError = 0;
  double prevError = 0;
  double prevErrorInertial = 0;
  double derivative;
  double derivativeInertial = 0;
  double limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double trackingWheel = fabs(tracker.rotation(deg));
  errorInertial = inertial_gyro.rotation(degrees);

  while (target > trackingWheel) {
    // Update sensor values
    trackingWheel = fabs(tracker.rotation(deg));
    errorInertial = inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 3;

    // Proportional
    error = target - trackingWheel;

    // Integral
    totalError += error;

    // introduces I term when needed
    if (error > 200) {
      totalError = 0;
    }

    if (fabs(error) < 200) {
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

    if (motorPower > 90) {
      motorPower = 90;
    }

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Tracker:");
    Brain.Screen.setCursor(4, 14);
    Brain.Screen.print(trackingWheel);

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
    prevErrorInertial = errorInertial;

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  printf("tracker%f\n", inertial_gyro.rotation(degrees));
  reset();
  setHold();
  setCoast();
}

int prevTarget = 0;
double prevTurn = 0;

// This function turns the robot right using a PID controller
// 45-90 5, 140, 8
void rightPID(double target, int counterThresh, double accuracy) {
  // Constants
  double kP = 1.4;
  double kI = 0.06;
  double kD = 5;

  int counter = 0;
  double error = 0;
  double totalError = 0;
  double prevError = 0;
  double targetError = 0;
  double derivative;
  int limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double inetVal = inertial_gyro.rotation(degrees);
  // targetError = prevTurn - prevTarget;
  // target = target - targetError;

  while (counter < counterThresh) {

    // Update sensor values
    // target = prevTarget;
    inetVal = inertial_gyro.rotation(degrees);

    // Update the limit
    limit += 5;

    // Proportional
    error = target - inetVal;

    // Integral
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
    printInet();
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Power:");
    Brain.Screen.setCursor(3, 14);
    Brain.Screen.print(motorPower);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Error:");
    Brain.Screen.setCursor(2, 14);
    Brain.Screen.print(error);
    if (fabs(error) <= 0.5) {
      counter += 1;
    }
    if (fabs(error) >= 0.5) {
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

void leftPID(double target, int counterThresh, double accuracy) {
  // Constants
  double kP = 1.4;
  double kI = 0.04;
  double kD = 5;

  int counter = 0;
  double error = 0;
  double totalError = 0;
  double prevError = 0;
  double targetError = 0;
  double derivative;
  int limit = 0;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  reset();
  double inetVal = inertial_gyro.rotation(degrees);
  // targetError = prevTurn - prevTarget;
  // target = target - targetError;

  while (counter < counterThresh) {

    // Update sensor values
    // target = prevTarget;
    inetVal = fabs(inertial_gyro.rotation(degrees));

    // Update the limit
    limit += 5;

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
    if (fabs(error) <= 0.3) {
      counter += 1;
    }
    if (fabs(error) >= 0.3) {
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

//This function turns the robot right using a PID controller
     void rightPID2 (int target){
       //Constants
      double kP = .77;
      double kI = 0.00000001;
      double kD = 0.002;  

      int error = 0;
      int totalError = 0;
      int prevError = 0;
      int derivative;
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int inetVal = inertial_gyro.rotation(degrees);

      while(target > inetVal){
        //Update sensor values
        inetVal = inertial_gyro.rotation(degrees);

        //Update the limit
        limit += 5;
        
        //Proportional
        error = target - inetVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);

        //If the motorPower is larger then the limit, the motor power will equal the limit
        if (limit < motorPower){
          motorPower = limit;
        }

        if (abs(motorPower) < 20){
          motorPower = 20;
        }

        //Sets the speed of the drive
        FL.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::fwd,motorPower*120,voltageUnits::mV); 


        prevError = error;
       
        

         task::sleep(10);
     }
    
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control slowly
      printInet();
      reset();
      setHold();
      setCoast();
      }

//This function turns the robot left using a PID controller slowly
     void leftPID2 (int target){
       //Constants
      double kP = .77;
      double kI = 0.00000001;
      double kD = 0.002;  

      int error = 0;
      int totalError = 0;
      int prevError = 0;
      int derivative;
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int inetVal = fabs(inertial_gyro.rotation(degrees));

      while(target > abs(inetVal)){
        //Update sensor values
        inetVal = fabs(inertial_gyro.rotation(degrees));

        //Update the limit
        limit += 5;
        
        //Proportional
        error = target - inetVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);

        //If the motorPower is larger then the limit, the motor power will equal the limit
        if (limit < motorPower){
          motorPower = limit;
        }

        if (abs(motorPower) <= 100){
          motorPower = 50;
        }

        //Sets the speed of the drive
        FL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::rev,motorPower*120,voltageUnits::mV); 


        prevError = error;
       
        

         task::sleep(10);
     }
    
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control slowly
      setHold();
      printInet();
      reset();
      setCoast();
      }