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
  //Goal 1
  index(60);
  forwardPID(1970,0,3,20);
  index(60);
  backwardPID(630,0,3,15);
  leftPID(62,3,1,100,1,.04, 5.5);
  forwardPID(800,-62,3,15);
  //goalScore
  task::sleep(100);
  forwardSpeed(30);
  allSpin(100);
  task::sleep(1500);
  allSpin(0);
  //goalScore
  allSpin(-100);
  //Goal 2
  backwardPID(1970,-62,3,15);
  leftPID(207,4,1,80,.8,.05, 5);
  index(60);
  forwardPID(840,-207,3,15);
  leftPID(109,4,1,90,1,.04, 5);
  index(60); 
  forwardPID(1170,-109,3,15);
  //goalScore
  task::sleep(100);
  forwardSpeed(20);
  allSpin(100);
  task::sleep(1500);
  allSpin(0);
  //goalScore
  backwardPID(300,-109,3,15);
  //Goal 3
  leftPID(199,4,1,90,1,.04,5);
  index(60);
  forwardPID(1300,-199,3,15);
  index(60);
  leftPID(215,3,1,90,1,.04,5);
  index(60);
  forwardPID(1000,-215,3,15);
  allSpin(100);
  task::sleep(500);
  index(60);
  backwardPID(500,-215,5,10);
  leftPID(155,4,1,90,1,.04,5);
  forwardPID(670,-155,3,15);
  //goalScore
  task::sleep(100);
  forwardSpeed(20);
  allSpin(100);
  task::sleep(1000);
  allSpin(0);
  //goalScore
  backwardPID(1900,-155,3,15);
  //Goal 4
  leftPID(285,4,1,80,.8,.05,5); 
  index(60);
  forwardPID(1080,-285,3,15);
  index(60);
  leftPID(197,4,1,90,1,.04,5);
  index(60);
  forwardPID(1100,-197,3,15);
  index(60);
  //goalScore
  task::sleep(100);
  forwardSpeed(10);
  allSpin(100);
  task::sleep(1000);
  allSpin(0);
  //goalScore
  backwardPID(780,-197,3,15);
  //Goal 5
  leftPID(287,3,1,287,1,.04,5);
  index(60);
  forwardPID(1600,-287,3,15);
  leftPID(187,3,1,90,1,.04,5);
  index(60);
  forwardPID(830,-187,3,15);
  index(60);
  backwardPID(430,-187,3,15);
  leftPID(248,3,1,90,1,.04,5);
  forwardPID(700,-248,3,15);
  //goalScore
  forwardSpeed(20);
  allSpin(100);
  task::sleep(1000);
  allSpin(0);
  //goalScore

  







  


  setCoast();
 
  
 }

// User control code
void usercontrol(void) {
  setCoast();
  task s = task(chassisMovement);
  task e = task(Intake);
  task x = task(sideRollers);
  while (true) {
    printTrackers();
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
