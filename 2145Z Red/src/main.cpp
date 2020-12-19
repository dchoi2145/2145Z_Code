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

 
//auton goes in here
void autonomous(void) { 
    //Red Right Side Home row
    flipOut();
    forwardPID(800); 
    conveyorSpeed(0);
    task::sleep(200);
    rightPID(60);
    task::sleep(300); 
    allSpin(100); 
    forwardPID(300);
    task::sleep(200); 
    allSpin(0); 
    backwardPID(300);
    task::sleep(400);
    allSpin(100); 
    rightPID(115);
    task::sleep(800);
    allSpin(0);
    backwardTime(50);
    task::sleep(600);
    backwardTime(0);
    task::sleep(400);
    forwardPID(2300);
    task::sleep(100);
    leftSlow(22);
    task::sleep(100);
    rollerSpeed(100);
    forwardTime(60);
    task::sleep(700);
    forwardTime(5);
    goalScore(60);
    task::sleep(100);
    backwardPID(300);

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
