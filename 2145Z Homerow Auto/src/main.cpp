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

  rollerSpeed(100);
  task::sleep(600);
  forwardPIDcurve(1100,0,3,30);
  createBallCountTask();
  forwardSpeed(80);
  task::sleep(200);
  goalScore(3,3);
  stopBallCountTask();
  allSpin(-30);
  backwardPID(1240,-30,60,80);
  rightPID(0,5,1,100,1.3,.8,5);
  allSpin(-100);
  backwardPID(1350,0,60,80);
  leftPID(90,3,1,90,1.1,.04,5);
  forwardPIDLight(1000,-90,3,20);
  createBallCountTask();
  forwardSpeed(20);
  goalScore(3,1);
  stopBallCountTask();
  backwardPID(600,-90,80,60);
  rightPID(0,4,2,90,1.1,.065,5);
  allSpin(-100);
  backwardPID(1800,0,30,30);
  leftPID(133.5,5,1,80,.8,.05,5);
  rollerSpeed(100);
  forwardPIDLight(1500,-135,30,30);
  createBallCountTask();
  forwardSpeed(20);
  goalScore(4,2);
  stopBallCountTask();
  allSpin(-30);
  backwardPID(1000,-133.5,80,60);










  







  


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
