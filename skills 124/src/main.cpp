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
  // Red Right Side Home row
  // 1st&2nd
  rollerSpeed(100);
  forwardPID(1550);
  leftPID(64.5, 1, 2);
  forwardPID(630);
  forwardSpeed(10);
  goalScore(70);
  forwardSpeed(0);
  task::sleep(200);
  // 3rd
  conveyorSpeed(100);
  backwardPID(500); 
  rollerSpeed(-100);
  conveyorSpeed(0);
  leftPID(134, 3, 1);
  task::sleep(200);
  /*backwardTime(40, 1200);
  task::sleep(500);*/
  rollerSpeed(100);
  forwardPID(4000);//4600
  rightPID(45, 3, 1);
  forwardPID(520);
  forwardSpeed(10);
  goalScore(70);
  forwardSpeed(0);
  // 4th
  task::sleep(200);
  allSpin(-100);
  backwardPID(530);
  leftPID(136.5, 3, 1);
  //backwardTime(40, 1000);
  rollerSpeed(100);
  forwardPID(2030);//used to be 2650
  rightPID(90.5, 3, 1);
  allSpin(100);
  task::sleep(1000);
  allSpin(0);
  // 5th
  leftPID(103, 5, 0.5);
  rollerSpeed(100);
  forwardPID(1700);
  rightPID(60, 5, 0.5);
  forwardPID(1000);
  forwardSpeed(10);
  goalScore(70);
  forwardSpeed(0);
  // 6th
  task::sleep(200);
  allSpin(-100);
  backwardPID(2000);
  leftPID(130, 8, 0.5);
  rollerSpeed(100);
  forwardPID(800);
  rightPID(86, 5, 0.5);
  forwardPID(1100);
  allSpin(100);
  task::sleep(800);
  allSpin(0);
  // 7th
  leftPID(90, 3, 1);
  rollerSpeed(100);
  forwardPID(2100);
  rightPID(47, 3, 1);
  forwardPID(500);
  forwardSpeed(10);
  goalScore(70);
  // 8th
  allSpin(-100);
  backwardPID(600);
  leftPID(135, 3, 1);
  //backwardTime(40, 1200);
  rollerSpeed(100);
  forwardPID(2050);
  rightPID(90, 3, 1);
  allSpin(100);
  task::sleep(800);
  allSpin(0);
  // 9th
  rightPID(178, 6, 0.5);
  rollerSpeed(100);
  forwardPID(1000);
  forwardTime(35,1000);
  allSpin(100);
  task::sleep(1000);
  allSpin(0);
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
