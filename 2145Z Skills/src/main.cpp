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
 
    //Skills
    flipOut();
    //first goal
    flipOut();
    rollerSpeed(100);
    forwardPID(800); 
    task::sleep(200);
    leftPID(29);
    task::sleep(500); 
    allSpin(100); 
    forwardPID(300);
    task::sleep(100); 
    allSpin(0);
    backwardPID(250);
    task::sleep(400);
    allSpin(100);  
    task::sleep(300);
    leftPID(85);
    task::sleep(700);
    allSpin(0);
    //second goal
    backwardTime(40);
    task::sleep(1000);
    backwardTime(0);
    task::sleep(300);
    rollerSpeed(100);
    forwardPID(2260);
    task::sleep(300);
    rightSlow(26);
    task::sleep(100);
    allSpin(100);
    forwardPID(320);
    task::sleep(200);
    allSpin(0);
    backwardPID(230);
    allSpin(100);
    task::sleep(300);
    allSpin(0);
    //third goal
    leftPID(78);
    task::sleep(700);
    backwardTime(40);
    task::sleep(1000);
    backwardTime(0);
    task::sleep(300);
    rollerSpeed(100);
    forwardPID(1300);
    task::sleep(200);
    rightPID(78);
    task::sleep(100); 
    allSpin(100);
    task::sleep(900);
    //fourth goal
    leftPID(107);
    task::sleep(300);
    rollerSpeed(100);
    forwardPID(400);
    task::sleep(200);
    rightPID(100);
    task::sleep(300);
    forwardPID(1210);
    task::sleep(200);
    allSpin(100);
    task::sleep(600);
    backwardPID(100);
    task::sleep(200);
    //fifth goal
    leftPID(90);
    task::sleep(200);
    backwardTime(30);
    task::sleep(800);
    backwardTime(0);
    task::sleep(300);
    rollerSpeed(100);
    forwardPID(1400);
    task::sleep(200);
    rightPID(90);
    task::sleep(200);
    allSpin(100);
    task::sleep(900);
    allSpin(0);
    //sixth goal
    rollerSpeed(100);
    task::sleep(200);
    leftPID(51);
    task::sleep(200); 
    forwardPID(1000);
    task::sleep(200);
    rightPID(36);
    task::sleep(200);
    forwardPID(200); 
    allSpin(100);
    task::sleep(500);
    allSpin(0);
    //seventh goal
    backwardPID(200);
    leftPID(77);
    task::sleep(1000);
    backwardTime(30);
    task::sleep(1200);
    backwardTime(0);
    task::sleep(400);
    rollerSpeed(100);
    forwardPID(1300);
    task::sleep(200);
    rightPID(80);
    allSpin(100);
    task::sleep(900);
    allSpin(0);
    //eigth goal
    backwardPID(100);
    task::sleep(200);
    leftPID(120);
    task::sleep(200);
    rollerSpeed(100);
    forwardPID(550);
    task::sleep(200);
    allSpin(100);
    task::sleep(200);
    allSpin(0);
    leftSlow(5);
    task::sleep(100); 
    rollerSpeed(-100);
    forwardTime(100);
    task::sleep(500);
    backwardTime(100);
    task::sleep(200);
    forwardTime(100);
    task::sleep(500);
    forwardTime(0);
    conveyorSpeed(50);
    task::sleep(2000);
    conveyorSpeed(0);

    

  }

// User control code 
void usercontrol(void) {
  flipOut();
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
