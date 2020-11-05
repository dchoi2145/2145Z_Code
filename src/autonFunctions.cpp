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

int error; //avgVal - target = position
int prevError; //last known position that is updated every 20 ms
int derivative; //error - prevError = speed
int totalError = 0; //totalError = totalError + error

void drivePID(int target){

  int leftVal = BL.position(degrees);
  int rightVal = BR.position(degrees);
  int avgVal = (leftVal + rightVal)/2;


  error = avgVal - target;

  derivative = error - prevError;

  totalError += error;

  double motorPower = (error *kP + derivative * kD + totalError* kI);

  FL.spin(fwd, motorPower, voltageUnits::volt);
  BR.spin(fwd, motorPower, voltageUnits::volt);
  FR.spin(fwd, motorPower, voltageUnits::volt);
  BL.spin(fwd, motorPower, voltageUnits::volt);


  prevError = error;
  vex::task::sleep(20);
}

