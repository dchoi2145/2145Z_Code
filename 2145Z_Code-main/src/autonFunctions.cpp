#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

//These are variables for sensor values
int lineSensorValue1 = ballDetector1.value(pct);
int lineSensorValue2 = ballDetector2.value(pct);
int lineSensorValue3 = ballDetector3.value(pct);
int opticalHue1 = optical1.hue();
   

    //This function initalizes the inertial sensor during pre auton
    void inertialCalibration(){
      inertial_gyro.calibrate();
      while (inertial_gyro.isCalibrating()){
      wait(2000, msec);
   }
   
}
    //This function resets all the values of the sensors
    void reset(){
      BL.resetRotation();
      BR.resetRotation();
      FL.resetRotation();
      FR.resetRotation();
      inertial_gyro.resetRotation();
}
    //This function allows our robot to move forwards for a certain amount of degrees and for a set speed
    void moveForward(int distance, int speed) {
  double wheelDiameterIN = 3.25;
  double travelTargetCM = distance; // this is the distance it goes which is set as a variable
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
    //This function allows our robot to move backwards for a certain amount of degrees and for a set speed
    void backward(int distance,int speed){
      while(true){
      FL.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      FR.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      BR.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      BL.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct); 
      }
}
    //This function allows our robot to move forwards for a certain amount of degrees and for a set speed
    void turnRight(int distance, int speed){
      FL.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      FR.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
      BR.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
      BL.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
}
    //This function allows our robot to move backwards for a certain amount of degrees and for a set speed
    void turnLeft(int distance,int speed){
      FL.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      FR.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
      BR.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
      BL.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct); 
}   
    //This function allows our robot to move at a specfic speed until told to stop
    void forwardTime(int speed){
      FL.spin(reverse, speed, pct);
      FR.spin(reverse, speed, pct);
      BL.spin(reverse, speed, pct);
      BR.spin(reverse, speed, pct);
    }
    //This function allows our robot to move at a specfic speed until told to stop
    void backwardTime(int speed){
      FL.spin(fwd, speed, pct);
      FR.spin(fwd, speed, pct);
      BL.spin(fwd, speed, pct);
      BR.spin(fwd, speed, pct);
    }
    //This function allows our robot to intake balls for a certain amount of degrees and for a set speed
    void intakeRollers(int distance, int speed){
      RightRoller.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      LeftRoller.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
    }
    //This function allows our robot to outtake balls for a certain amount of degrees and for a set speed
    void outtakeRollers(int distance, int speed){
      RightRoller.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
      LeftRoller.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
    }
    //This function allows our robot to intake and score balls for a certain amount of degrees and for a set speed
    void intakeConveyor(int distance, int speed){
      Conveyor1.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
      Conveyor2.rotateFor(distance,rotationUnits::deg,speed,velocityUnits::pct);
    }
    //This function allows our robot to downttake and get rid of balls for a certain amount of degrees and for a set speed
    void outtakeConveyor(int distance, int speed){
      Conveyor1.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
      Conveyor2.rotateFor(-distance,rotationUnits::deg,speed,velocityUnits::pct);
    }
    //This function allows our robot to set the speed of the rollers
    void rollerSpeed(int speed){
      RightRoller.spin(reverse, speed, pct);
      LeftRoller.spin(reverse, speed, pct);
    }
    //This function allows our robot to set the speed of the conveyor
    void conveyorSpeed(int speed){
      Conveyor1.spin(fwd, speed, pct);
      Conveyor2.spin(fwd, speed, pct);
    }
    //This function allows for all intaking subsystems to spin
    void allSpin (int speed){
      Conveyor1.spin(fwd, speed, pct);
      Conveyor2.spin(fwd, speed, pct);
      RightRoller.spin(reverse, speed, pct);
      LeftRoller.spin(reverse, speed, pct);
    }

    //This function stops red balls at the top of the conveyor for optimal priming position
    void ballStopRed(){
      if (lineSensorValue2 <= 60 || lineSensorValue3 <= 60 || opticalHue1 < 40){
        Conveyor1.stop(hold);
        Conveyor2.stop(hold);
        RightRoller.stop(coast);
        LeftRoller.stop(coast);
    }
  }        
    //This function stops blue balls at the top of the conveyor for optimal priming position
    void ballStopBlue(int speed){
      while  (lineSensorValue2 > 60 && lineSensorValue3 > 60 /*|| opticalHue1 < 100*/){
        Conveyor1.spin(fwd, speed, pct);
        Conveyor2.spin(fwd, speed, pct);
        RightRoller.spin(reverse, speed, pct);
        LeftRoller.spin(reverse, speed, pct);
        printLineValue1();
        printLineValue2();
        printOptical();
      } 
        Conveyor1.stop(hold);
        Conveyor2.stop(hold);
        RightRoller.stop(coast);
        LeftRoller.stop(coast);
      }
    

    //This function stops all balls at the top of the conveyor for optimal priming position
    void ballStopAll(){
      if (lineSensorValue2 <= 60 || lineSensorValue3 <= 60 || opticalHue1 < 40 || opticalHue1 > 100){
        Conveyor1.stop(hold);
        Conveyor2.stop(hold);
    }
  }
    //Sets the conveyor to coast after being set to hold
    void setCoast(){
        FL.stop(coast);
        FR.stop(coast);
        BL.stop(coast);
        BR.stop(coast);
        Conveyor1.stop(coast);
        Conveyor2.stop(coast);
    }
    void setHold(){
        FL.stop(hold);
        FR.stop(hold);
        BL.stop(hold);
        BR.stop(hold);
    }
    void flipOut(){
        conveyorSpeed(100);
        rollerSpeed(100);
        task::sleep(200);
        rollerSpeed(0);
    }

    //This function moves the robot forwards using a PID controller
    void forwardPID (int target){
      //Constants
      double kP = 0.24;
      double kPAngle = .1;
      double kI = 0;
      double kD = 0.2;
      
      int error = 0;
      int totalError = 0;
      int prevError = 0; 
      int derivative; 
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int leftVal = BL.position(degrees);
      int rightVal = BR.position(degrees);
      int avgVal = (abs(leftVal) + abs(rightVal))/2;
      int inetVal = inertial_gyro.rotation(degrees);

      while(target > avgVal){
        //Update sensor values
        leftVal = BL.position(degrees);
        rightVal = BR.position(degrees);
        avgVal = (abs(leftVal) + abs(rightVal))/2;
        inetVal = inertial_gyro.rotation(degrees);

        if (inetVal < 0){
          inetVal = (inetVal + 360)*-1;
        }
        
        printInet();
      
        //Update the limit
        limit += 5;

        //Proportional
        error = target - avgVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        //Find the speed of chassis based of the sum of the constants
        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);
      
        //If the motorPower is larger then the limit, the motor power will equal the limit
        if (limit < motorPower){
          motorPower = limit;
        }

        if (abs(motorPower) < 5){
          motorPower = 5;
        }

        if (motorPower > 90){
          motorPower = 90;
        }

        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("Power:");
        Brain.Screen.setCursor(3,14);
        Brain.Screen.print(motorPower);



        //Sets the speed of the drive
        FL.spin(directionType::rev,110*(motorPower -  inetVal * kPAngle),voltageUnits::mV);
        BL.spin(directionType::rev,110*(motorPower -  inetVal * kPAngle),voltageUnits::mV);
        FR.spin(directionType::rev,110*(motorPower +  inetVal * kPAngle),voltageUnits::mV);
        BR.spin(directionType::rev,110*(motorPower +  inetVal * kPAngle),voltageUnits::mV); 

  
        prevError = error;

         task::sleep(10);
    }
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control
      setHold();
      setCoast();
     }
     //This function moves the robot forwards using a PID controller slowly
    void forwardSlowPID (int target){
      //Constants
      double kP = 0.25;
      double kI = 0;
      double kD = 0.2;
      
      int error = 0;
      int totalError = 0;
      int prevError = 0; 
      int derivative; 
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int leftVal = BL.position(degrees);
      int rightVal = BR.position(degrees);
      int avgVal = (abs(leftVal) + abs(rightVal))/2;

      while(target > avgVal){
        //Update sensor values
        leftVal = BL.position(degrees);
        rightVal = BR.position(degrees);
        avgVal = (abs(leftVal) + abs(rightVal))/2;
      
        //Update the limit
        limit += 5;

        //Proportional
        error = target - avgVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        //Find the speed of chassis based of the sum of the constants
        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);
      
        //If the motorPower is larger then the limit, the motor power will equal the limit
        if (limit < motorPower){
          motorPower = limit;
        }

        if (abs(motorPower) < 5){
          motorPower = 12;
        }

        //Sets the speed of the drive
        FL.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::rev,motorPower*120,voltageUnits::mV); 

  
        prevError = error;

         task::sleep(10);
    }
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control
      setHold();
      setCoast();
     }
     //This function moves the robot backwards using a PID controller
    void backwardPID (int target){
      //Constants
      double kP = 0.238;
      double kI = 0;
      double kD = 0.2;
      
      int error = 0;
      int totalError = 0;
      int prevError = 0; 
      int derivative; 
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int leftVal = BL.position(degrees);
      int rightVal = BR.position(degrees);
      int avgVal = (abs(leftVal) + abs(rightVal))/2;

      while(target > avgVal){
        //Update sensor values
        leftVal = BL.position(degrees);
        rightVal = BR.position(degrees);
        avgVal = (abs(leftVal) + abs(rightVal))/2;
      
        //Update the limit
        limit += 5;

        //Proportional
        error = target - avgVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        //Find the speed of chassis based of the sum of the constants
        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);
      
        //If the motorPower is larger then the limit, the motor power will equal the limit
        if (limit < motorPower){
          motorPower = limit;
        }
        if (abs(motorPower) < 5){
          motorPower = 5;
        }

        //Sets the speed of the drive
        FL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::fwd,motorPower*120,voltageUnits::mV); 

  
        prevError = error;

         task::sleep(10);
    }
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control
      setHold();
      setCoast();
     }

     //This function moves the robot backwards using a PID controller slowly
    void backwardSlowPID (int target){
      //Constants
      double kP = 0.25;
      double kI = 0;
      double kD = 0.2;
      
      int error = 0;
      int totalError = 0;
      int prevError = 0; 
      int derivative; 
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int leftVal = BL.position(degrees);
      int rightVal = BR.position(degrees);
      int avgVal = (abs(leftVal) + abs(rightVal))/2;

      while(target > avgVal){
        //Update sensor values
        leftVal = BL.position(degrees);
        rightVal = BR.position(degrees);
        avgVal = (abs(leftVal) + abs(rightVal))/2;
      
        //Update the limit
        limit += 5;

        //Proportional
        error = target - avgVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        //Find the speed of chassis based of the sum of the constants
        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);
      
        //If the motorPower is larger then the limit, the motor power will equal the limit
        if (limit < motorPower){
          motorPower = limit;
        }
        if (abs(motorPower) < 5){
          motorPower = 12;
        }

        

        //Sets the speed of the drive
        FL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::fwd,motorPower*120,voltageUnits::mV); 

  
        prevError = error;

         task::sleep(10);
    }
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control
      setHold();
      setCoast();
     }

     //This function turns the robot right using a PID controller
     void rightPID (int target){
       //Constants
      double kP = .98;
      double kI = 0;
      double kD = 0.1;  

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

        if (abs(motorPower) < 5){
          motorPower = 5;
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
      setHold();
      setCoast();
      }

      void rightSlowPID (int target){
       //Constants
      double kP = 1;
      double kI = 0;
      double kD = 0.1;  

      int error = 0;
      int totalError = 0;
      int prevError = 0;
      int derivative;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int inetVal = inertial_gyro.rotation(degrees);

      while(target > inetVal){
        //Update sensor values
        inetVal = inertial_gyro.rotation(degrees);
        
        //Proportional
        error = target - inetVal;

        //Integral
        totalError += error;

        //Derivative
        derivative = error-prevError;

        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);

        if (abs(motorPower) < 5){
          motorPower = 15;
        }

        //Sets the speed of the drive
        FL.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::fwd,motorPower*120,voltageUnits::mV); 


        prevError = error;
       
        

         task::sleep(10);
     }
    
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control
      setHold();
      setCoast();
      }
     
     //This function turns the robot left using a PID controller slowly
     void leftPID (int target){
       //Constants
      double kP = .98;
      double kI = 0;
      double kD = 0.1;  

      int error = 0;
      int totalError = 0;
      int prevError = 0;
      int derivative;
      int limit = 0;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int inetVal = inertial_gyro.rotation(degrees);

      while(target > abs(inetVal)){
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

        if (abs(motorPower) < 5){
          motorPower = 5;
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
      setCoast();
      }

      void leftSlowPID (int target){
       //Constants
      double kP = 1;
      double kI = 0;
      double kD = 0.1;  

      int error = 0;
      int totalError = 0;
      int prevError = 0;
      int derivative;

      //Resets the sensor values and then sets the current sensor values to the sensors
      reset();
      int inetVal = inertial_gyro.rotation(degrees);

      while(target > abs(inetVal)){
        //Update sensor values
        inetVal = inertial_gyro.rotation(degrees);
        
        //Proportional
        error = target - inetVal;

        //Integral 
        totalError += error;

        //Derivative
        derivative = error-prevError;

        int motorPower = (kP*error) + (kI*totalError) + (kD*derivative);

        if (abs(motorPower) < 5){
          motorPower = 15;
        }

        //Sets the speed of the drive
        FL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        BL.spin(directionType::fwd,motorPower*120,voltageUnits::mV);
        FR.spin(directionType::rev,motorPower*120,voltageUnits::mV);
        BR.spin(directionType::rev,motorPower*120,voltageUnits::mV); 


        prevError = error;
       
        

         task::sleep(10);
     }
    
      //When the loop ends, the motors are set to brake for less uncertainty and then set the coast for drive control
      setHold();
      setCoast();
      }
    

    
