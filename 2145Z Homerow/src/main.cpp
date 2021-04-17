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
  //Goal 1
  index(60, 100);
  forwardPID(2000,0,3,20);
  index(60, 100);
  backwardPID(600,0,3,15);
  leftPID(62,3,1,100,1,.04, 5.5);
  index(100, 10);
  forwardPIDLight(1100,-62,3,15);
  task::sleep(200);
  forwardSpeed(20);
  goalScore(100,1,0);
  allSpin(-100);

  //Goal 2
  backwardPID(1880,-62,3,15);
  leftPID(207,4,1,80,.8,.05, 5);
  index(60,100);
  forwardPID(970,-207,3,15);
  leftPID(109,4,1,90,1,.04, 5);
  index(60,100); 
  forwardPIDLight(1570,-109,3,15);
  task::sleep(100);
  forwardSpeed(20);;
  goalScore1();
  backwardPID(300,-109,3,15);
  
  //Goal 3
  leftTurn(79,4,1,90,60);
  allSpin(-100);
  task::sleep(400);
  leftPID(199,4,1,90,1,.04,5.5);
  index(60,100);
  forwardPID(1300,-199,3,15);
  index(60,100);
  leftPID(215,3,1,90,1,.04,5);
  index(60,100);
  forwardPID(930,-215,3,15);
  index(100,30);
  backwardPID(390,-215,5,10);
  leftPID(155,4,1,90,1,.04,5);
  forwardPIDLight(900,-155,3,15);
  task::sleep(100);
  forwardSpeed(20);
  goalScore(100,1,0);
  allSpin(-100);
  backwardPID(1970,-155,3,30);

  //Goal 4
  leftPID(285,4,1,80,.8,.05,5); 
  index(60,100);
  forwardPID(910,-285,3,15);
  index(60,100);
  leftPID(197,4,1,90,1,.04,5);
  index(60,100);
  forwardPIDLight(1500,-197,3,15);
  index(100,30);
  task::sleep(100);
  forwardSpeed(20);;
  goalScore1();
  allSpin(-100);
  backwardPID(780,-197,3,15);

  //Goal 5
  leftPID(287,3,1,287,1,.04,5);
  index(60,100);
  forwardPID(1600,-287,3,15);
  leftPID(187,3,1,90,1,.04,5);
  index(60,100);
  forwardPID(830,-187,3,15);
  index(100,30);
  backwardPID(520,-187,3,15);
  leftPID(248,3,1,90,1,.04,5);
  forwardPIDLight(1000,-248,3,15);
  task::sleep(100);
  forwardSpeed(20);
  goalScore(100,1,0);
  allSpin(-100);
  backwardPID(1550,-248,3,30);

  //Goal 6
  leftPID(383,3,1,80,.8,.05,5);
  index(60,100);
  forwardPID(1100,-383,3,30);
  index(100,30);
  leftPID(292,3,1,90,1,.04,5);
  forwardPIDLight(1500,-292,3,30);
  task::sleep(100);
  forwardSpeed(20);;
  goalScore1();
  backwardPID(290,-292,3,30);

  //goal 7
  leftPID(400,3,1,90,1,.04,5); 
  allSpin(-100);
  task::sleep(200);
  leftPID(380,3,1,90,1,.04,5); 
  index(60,100);
  forwardPID(1300,-380,3,30);
  index(60,100);
  leftPID(393,3,1,90,1,.04,5);
  index(60,100);
  forwardPID(1000,-392,3,30);
  index(60,100);
  backwardPID(560,-393,5,30);
  leftPID(338,3,1,90,1,.04,5);
  forwardPIDLight(1200,-338,3,30);
  task::sleep(100);
  forwardSpeed(20);
  goalScore(100,1,0);
  allSpin(-100);
  backwardPID(600,-338,3,30);

  //Goal 8
  leftPID(464,3,1,80,.8,.05,5);
  index(60,100);
  forwardPID(1680,-464,3,30);
  index(60,100);
  task::sleep(300);
  leftPID(376,3,1,90,1,.04,5);
  index(100,30);
  forwardPIDLight(500,-376,3,1);
  task::sleep(100);
  forwardSpeed(20);;
  goalScore1();
  task::sleep(200);
  allSpin(-50);
  backwardPID(500,-376,3,30);

  //Goal 9
  leftPID(198,3,1,80,.8,.05,5);
  index(60,100);
  forwardPID(1700,-197,3,30);
  index(60,100);
  rollerSpeed(-20);
  forwardSpeed(80);
  task::sleep(700);
  rollerSpeed(-100);
  






  







  


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
