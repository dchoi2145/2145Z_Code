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
  

  void gyroTurn (int target) {
    double kP = .2;
    double kI = 0;
    double kD = 0;
    int error = 0;
    int totalError = 0;
    int prevError = 0;
    int derivative;
    int acceptableErrorRange = 0;
    

     while(abs(target - inertial_gyro.rotation(degrees)) > acceptableErrorRange) {
          inertial_gyro.resetRotation();
          prevError = error;
          

          //Proportional
          error = target - inertial_gyro.rotation(degrees);
          //Integral
          totalError += error; 
          //Derivative
          derivative = error - prevError;

          int motorPower = error * kP +  kD * derivative + kI * totalError;
       if(inertial_gyro.rotation(degrees) < target){
          FL.spin(fwd, motorPower, velocityUnits::pct);
          FR.spin(fwd, -motorPower, velocityUnits::pct);
          BL.spin(reverse, -motorPower, velocityUnits::pct);
          BR.spin(reverse, motorPower, velocityUnits::pct);
       }
       else if(inertial_gyro.rotation(degrees) > target){
          FL.spin(fwd,-motorPower, velocityUnits::pct);
          FR.spin(fwd, motorPower, velocityUnits::pct);
          BL.spin(reverse, motorPower, velocityUnits::pct);
          BR.spin(reverse, -motorPower, velocityUnits::pct);
       }
       else{
          FR.stop(hold);
          FL.stop(hold);
          BR.stop(hold);
          BL.stop(hold);
       }
         
     }
          FR.stop(hold);
          FL.stop(hold);
          BR.stop(hold);
          BL.stop(hold);
  }
     void forwardPID (int target){
      double kP = 0.1;
      double kI = 0.1;
      double kD = 0.2;
      int error = 0;
      int totalError = 0;
      int prevError = 0;
      int derivative;

      reset();
      int leftVal = BL.position(degrees);
      int rightVal = BR.position(degrees);
      int avgVal = (abs(leftVal) + abs(rightVal))/2;

      while(target > avgVal){
        leftVal = BL.position(degrees);
        rightVal = BR.position(degrees);
      
        avgVal = (abs(leftVal) + abs(rightVal))/2;
        //Proportional
        error = target - avgVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);
        FL.spin(directionType::rev,motorPower,velocityUnits::pct);
        BL.spin(directionType::rev,motorPower,velocityUnits::pct);
        FR.spin(directionType::rev,motorPower,velocityUnits::pct);
        BR.spin(directionType::rev,motorPower,velocityUnits::pct); 

         task::sleep(10);
    }

      FL.stop(coast);
      FR.stop(coast);
      BL.stop(coast);
      BR.stop(coast);

     }

    

  
