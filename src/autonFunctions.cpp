#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

void inertialCalibration(){
  inertial_gyro.calibrate();
   while (inertial_gyro.isCalibrating()){
     wait(100, msec);
   }
   
}

void reset(){
  BL.resetRotation();
  BR.resetRotation();
  FL.resetRotation();
  FR.resetRotation();
  inertial_gyro.resetRotation();
}
//Constants
double kP=0.1;
double kI=0;
double kD=0;  
double turnkP=0;
double turnkI=0;
double turnkD=0; 

int target = 10;
int turnTarget = 0;

int error; //avgVal - target = position
int prevError = 0; //last known position that is updated every 20 ms
int derivative; //error - prevError = speed
int totalError = 0; //totalError = totalError + error

int turnError; //avgVal - target = position
int turnPrevError = 0; //last known position that is updated every 20 ms
int turnDerivative; //error - prevError = speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDrive = false;

bool enableDrivePID = true;

int drivePID(){

  while(enableDrivePID){

  if (resetDrive){
    resetDrive = false;
    FL.setPosition(0,degrees);
    BR.setPosition(0,degrees);
    FR.setPosition(0,degrees);
    BL.setPosition(0,degrees);
  }

  int leftVal = BL.position(degrees);
  int rightVal = BR.position(degrees);
  int avgVal = (leftVal + rightVal)/2;

  //Proportional
  error = avgVal - target;
  //Derivative
  derivative = prevError - error;
  //Integral
  totalError += error;

  double motorPower = error * kP + derivative * kD + totalError* kI;

  //Turn

  int turnAvgVal = leftVal - rightVal;

  //Proportional
  turnError = turnAvgVal - turnTarget;
  //Derivative
  turnDerivative = turnError - turnPrevError;
  //Integral
  turnTotalError += turnError;

  double turnMotorPower = (turnError *turnkP + turnDerivative * turnkD + turnTotalError* turnkI);


  FL.spin(fwd, motorPower + turnMotorPower, voltageUnits::volt);
  BR.spin(fwd, motorPower - turnMotorPower, voltageUnits::volt);
  FR.spin(fwd, motorPower + turnMotorPower, voltageUnits::volt);
  BL.spin(fwd, motorPower - turnMotorPower, voltageUnits::volt);


  prevError = error;
  turnPrevError = turnError;
  vex::task::sleep(20);


  }
   return 1;
}
