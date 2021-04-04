#include "vex.h"

#include "driverFunctions.h"

#include "autonFunctions.h"

using namespace vex;

  int lineSensor1Value = 0;
  int lineSensor2Value = 0;
  int lineSensor3Value = 0;
  int optical1Hue = 0;
  int inetVal = 0;

//code for drive
int chassisMovement(){
  while(true) {

    //driving left side quadtratically 
    if(Controller1.Axis3.value()>0){ 
      FL.spin(reverse, abs(Controller1.Axis3.value()), pct);//140
      BL.spin(reverse, abs(Controller1.Axis3.value()), pct);
    }
    else{ 
      FL.spin(fwd, abs(Controller1.Axis3.value()), pct);
      BL.spin(fwd, abs(Controller1.Axis3.value()), pct);
    }
     
     //driving right side quadtratically 
    if(Controller1.Axis2.value()>0){
      FR.spin(reverse, Controller1.Axis2.value(), pct);
      BR.spin(reverse, Controller1.Axis2.value(), pct);
    }
    else{
      FR.spin(fwd, pow(Controller1.Axis2.value(), 2)/100, pct);
      BR.spin(fwd, pow(Controller1.Axis2.value(), 2)/100, pct);//140
    }
    
    task::sleep(10);
  
  }
}
  
  //all intake control functions
  int Intake() {
  while (true) {
    if(Controller1.ButtonL1.pressing()){
        Conveyor1.spin(fwd, 100, pct);
    }
    else if(Controller1.ButtonL2.pressing()){
        Conveyor2.spin(fwd, 100, pct);
    }
    else if(Controller1.ButtonL1.pressing()&&Controller1.ButtonL2.pressing()){
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }
    else if(Controller1.ButtonRight.pressing()){
       Conveyor1.spin(reverse, 100, pct);
       Conveyor2.spin(reverse, 100, pct);
    }
    else if(Controller1.ButtonR1.pressing()){
      RightRoller.spin(fwd, 100, pct);
      LeftRoller.spin(fwd, 100, pct);
    }
    else if(Controller1.ButtonR2.pressing()){
      RightRoller.spin(reverse, 100, pct);
      LeftRoller.spin(reverse, 100, pct);
    }
    
      else {
        LeftRoller.spin(fwd, 0, pct);
        RightRoller.spin(fwd, 0, pct);
        Conveyor1.spin(fwd, 0, pct);
        Conveyor2.spin(fwd, 0, pct);
        LeftRoller.stop(coast);
        RightRoller.stop(coast);
        Conveyor1.stop(coast);
        Conveyor2.stop(coast);
    }
  
    task::sleep(10);
    }
  }
  //prints value of line sensor to screen
   void printLineValue1() {
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("Line Sensor1:");
      Brain.Screen.setCursor(1,14);
      Brain.Screen.print(ballDetector1.value(pct));

  }
  //prints value of line sensor to screen
   void printLineValue2(){
      Brain.Screen.setCursor(2,1);
      Brain.Screen.print("Line Sensor2:");
      Brain.Screen.setCursor(2,14);
      Brain.Screen.print(ballDetector2.value(pct));

   }

   void printOptical(){ 
      Brain.Screen.setCursor(3,1);
      Brain.Screen.print("Optical:");
      Brain.Screen.setCursor(3,14);
      Brain.Screen.print(optical1.hue());
   }

   void opticalLight(){
     optical1.setLightPower(100);
   }
   
   void printInet(){
      Brain.Screen.setCursor(4,1);
      Brain.Screen.print("Inertial:");
      Brain.Screen.setCursor(4,14);
      Brain.Screen.print(inertial_gyro.rotation(degrees));
   }
   
   
    