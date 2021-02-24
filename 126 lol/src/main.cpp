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
  allSpin(30);
  forwardPID(2070);
  task::sleep(100);
  allSpin(0);
  rollerSpeed(100); 
  backwardPID(570);
  leftPID(63,5,.8);
  forwardPID(700);
  forwardSpeed(10);
  allSpin(100);
  task::sleep(1500);
  allSpin(-100);
  backwardPID(700);
  leftPID(158,5,.5);
  rollerSpeed(100);
  forwardPID(2160);
  rightPID(116,5,.5);
  forwardPID(1300);
  allSpin(100);
  forwardSpeed(5);
  task::sleep(1500);
  allSpin(0);
  forwardSpeed(0);
  



  
  
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
