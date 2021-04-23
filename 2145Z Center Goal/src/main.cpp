/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Daquan Fisher                                             */
/*    Created:      September 2020 - April 2021                               */
/*    Description:  2145Z Competition Code                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include "autonFunctions.h"

#include "driverFunctions.h"

using namespace vex;

competition Competition;

// Initializing Robot Configuration
void pre_auton(void) {

  vexcodeInit();
  inertialCalibration();
  
}
 
// auton goes in here
void autonomous(void) {
  backwardPID(1500,0,3,30);
 /* rollerSpeed(100);
  forwardPID(1530,0,3,30);
  task::sleep(300);
  backwardPID(850,-20,3,30);
  rightPID(45,3,1,90,1.3,.05,5);
  rollerSpeed(0);
  forwardPID(1300,45,3,30);
  task::sleep(700);
  forwardSpeed(1);
  task::sleep(1000);
  allSpin(100);
  rollerSpeed(0);
  task::sleep(1000);
  allSpin(0);
  backwardPID(1000,45,3,30);*/








  







  


  setCoast();
 
  
 }

// User control code
void usercontrol(void) {
  setCoast();
  task s = task(chassisMovement);
  task e = task(Intake);
  task x = task(sideRollers);
  while (true) {
    printLineValue1();

    
   

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
