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
  opticalLight();
}
 
// auton goes in here
void autonomous(void) {
    
  rollerSpeed(100);
  task::sleep(300);
  forwardPID(1590);
  task::sleep(200);
  leftPID(61, 3,.7);
  forwardPID(500);
  allSpin(100);
  task::sleep(900);
  reset();
  backwardPID(700);
  leftPID(136.8,5,.5);
  allSpin(0);
  task::sleep(200);
  forwardPID(3400);
  rightPID(45,5,1);
  task::sleep(200);
  rollerSpeed(100);
  forwardPID(750);
  forwardSpeed(30);
  task::sleep(500);
  goalScore(100,2);
  backwardPID(1000);
  setCoast();

  
  
 }

// User control code
void usercontrol(void) {
  task p = task(chassisMovement);
  task i = task(Intake);
  while (true) {
    opticalLight();
    printLineValue1();
    printLineValue2();
    printOptical();
    printInet();
    
   

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
