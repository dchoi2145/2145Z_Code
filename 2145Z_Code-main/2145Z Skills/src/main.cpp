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
    /*flipOut();
    task::sleep(100);
    forwardPID(760);
    conveyorSpeed(0);
    task::sleep(200);
    rightPID(62);
    allSpin(100);
    task::sleep(200); 
    forwardTime(30);
    task::sleep(1000); 
    forwardTime(0);
    task::sleep(1000);
    allSpin(0);
    backwardPID(400);
    allSpin(100);
    task::sleep(400);
    allSpin(0);
    rightPID(115);
    task::sleep(200);
    backwardTime(50);
    task::sleep(900);
    backwardTime(0);
    task::sleep(200);
    forwardPID(2180);
    leftPID(21);
    task::sleep(200);
    forwardTime(30);
    allSpin(100);
    task::sleep(2000);
    forwardTime(0);
    allSpin(0);*/

    //Red Right Side 3 ball
    /*flipOut();
    task::sleep(100);
    forwardPID(760); 
    conveyorSpeed(0);
    task::sleep(200);
    rightPID(62);
    allSpin(100);
    task::sleep(200); 
    forwardTime(30);
    task::sleep(1000); 
    forwardTime(0);
    task::sleep(1000);
    allSpin(0);
    backwardPID(730);
    allSpin(100);
    task::sleep(400);
    allSpin(0);
    leftPID(38);
    task::sleep(500);
    forwardPID(700);
    task::sleep(200);
    forwardTime(20); 
    task::sleep(800);
    allSpin(100);
    forwardTime(0);  
    task::sleep(2000);
    allSpin(0);*/

    //Skills Red Right Side
    /*flipOut();
    task::sleep(100);
    allSpin(100);
    forwardPID(800); 
    task::sleep(200);
    rightPID(60);
    allSpin(0);
    task::sleep(200); 
    forwardPID(340);
    conveyorSpeed(100);
    task::sleep(1000); 
    conveyorSpeed(0);
    backwardPID(730);
    task::sleep(400);
    leftPID(45);
    task::sleep(500);
    rollerSpeed(100);
    forwardPID(700);
    task::sleep(200);
    rightSlowPID(51);
    task::sleep(200);
    forwardSlowPID(100);
    allSpin(100); 
    task::sleep(1000);
    backwardSlowPID(260);
    task::sleep(300);
    leftSlowPID(34);
    task::sleep(300);
    rollerSpeed(100);
    forwardPID(1300);
    rollerSpeed(0);*/
    moveForward(10,30);




    

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
