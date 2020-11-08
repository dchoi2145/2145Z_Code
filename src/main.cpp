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


//auton goes in here
void autonomous(void) {
    /*vex::task forward(drivePID);
    resetDrive = true;
    target = 100;
    turnTarget = 0;

 vex::task::sleep(100);

    resetDrive = true;
    target = 100;
    turnTarget = 90;*/

  }

// User control code 
void usercontrol(void) {
      //enableDrivePID = false;
  while (true) {
    printLineValue1();
    printLineValue2();
    task p = task(chassisMovement);
    task i = task(Intake);
  

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
