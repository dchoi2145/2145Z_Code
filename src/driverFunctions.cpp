#include "vex.h"

#include "driverFunctions.h"

#include "autonFunctions.h"

using namespace vex;

//bool YPressed = true;
int lineSensor1Value = 0;
  int lineSensor2Value = 0;
void autoGroup(){
    LeftRoller.spin(fwd, 100, pct);
    RightRoller.spin(fwd, 100, pct);
    Conveyor1.spin(reverse, 100, pct);
    Conveyor2.spin(reverse, 100, pct);
    task::sleep(500);
    if (lineSensor2Value < 70){
      Conveyor1.spin(fwd, 100, pct);
      Conveyor2.spin(fwd, 100, pct);
    }
    else{
      Conveyor1.stop(coast);
      Conveyor2.stop(coast);
    }
}

//code for drive
int chassisMovement(){
  while(true) {

    //driving left side quadtratically 
    if(Controller1.Axis3.value()>0){
      FL.spin(reverse, pow(Controller1.Axis3.value(), 2)/100, pct);
      BL.spin(reverse, pow(Controller1.Axis3.value(), 2)/100, pct);
    }
    else{
      FL.spin(fwd, pow(Controller1.Axis3.value(), 2)/100, pct);
      BL.spin(fwd, pow(Controller1.Axis3.value(), 2)/100, pct);
    }
     
     //driving right side quadtratically 
    if(Controller1.Axis2.value()>0){
      FR.spin(reverse, pow(Controller1.Axis2.value(), 2)/100, pct);
      BR.spin(reverse, pow(Controller1.Axis2.value(), 2)/100, pct);
    }
    else{
      FR.spin(fwd, pow(Controller1.Axis2.value(), 2)/100, pct);
      BR.spin(fwd, pow(Controller1.Axis2.value(), 2)/100, pct);
    }

    task::sleep(10);
  
  }
}
  
  //all intake control functions
  int Intake() {
  while (true) {
     lineSensor1Value = ballDetector1.value(pct);
     lineSensor2Value = ballDetector2.value(pct);

      if (Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()) {
        LeftRoller.spin(fwd, 100, pct);
        RightRoller.spin(fwd, 100, pct);
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }

      else if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) {
        LeftRoller.spin(reverse, 100, pct);
        RightRoller.spin(reverse, 100, pct);
        Conveyor1.spin(reverse, 100, pct);
        Conveyor2.spin(reverse, 100, pct);
    }

      else if (Controller1.ButtonL1.pressing()) {
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }

      else if (Controller1.ButtonL2.pressing()) {
        Conveyor1.spin(reverse, 100, pct);
        Conveyor2.spin(reverse, 100, pct);
    }

      /*else if (lineSensor1Value <= 69){
        Conveyor1.spin(fwd, 100, pct);
        Conveyor2.spin(fwd, 100, pct);
    }*/

      else if (Controller1.ButtonR2.pressing()) { 
        LeftRoller.spin(reverse, 100, pct);
        RightRoller.spin(reverse, 100, pct);
    }

      else if (Controller1.ButtonR1.pressing()) {
        LeftRoller.spin(fwd, 100, pct);
        RightRoller.spin(fwd, 100, pct);
    }
    
      else {
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
   
   
    