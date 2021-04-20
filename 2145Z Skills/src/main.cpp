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
  backwardPID(570,0,3,30);
  leftPID(62,3,1,100,1,.04, 5.5);
  index(100, 10);
  createBallCountTask();
  forwardPIDLight(1100,-62,3,30);
  forwardSpeed(20);
  goalScore();
  stopBallCountTask();

  //Goal 2
  rollerSpeed(50);
  conveyorBottom(-100);
  backwardPID(1950,-62,3,30);
  allSpin(-100);
  task::sleep(400);
  leftPID(207,4,1,80,.8,.05, 5);
  index(60,100);
  forwardPID(935,-207,3,15);
  leftPID(109,3,1,90,1,.04, 5.5);
  index(100,100); 
  forwardPIDLight(1570,-109,3,15);
  rollerSpeed(-100);
  task::sleep(300);
  forwardSpeed(20);
  goalScore1();
  conveyorBottom(-100);
  backwardPID(400,-109,3,15);
  conveyorBottom(-100);
  
  //Goal 3
  leftPID(225,4,1,80,.9,.04, 6.5);
  allSpin(-100);
  task::sleep(400);
  leftPID(199,4,1,90,1,.02,8);
  index(60,100);
  forwardPID(1730,-199,3,30);
  index(60,100);
  leftPID(155,4,1,90,1,.04,5);
  createBallCountTask();
  forwardPIDLight(900,-155,3,15);
  forwardSpeed(20);
  task::sleep(200);
  goalScore();
  stopBallCountTask();
  //rollerSpeed(100);
  conveyorBottom(-100);
  backwardPID(2000,-155,3,30);
  allSpin(-100);
  task::sleep(200);

  //Goal 4
  leftPID(285,4,1,80,.8,.05,5); 
  index(60,100);
  forwardPID(960,-285,3,15);
  index(60,100);
  leftPID(197,4,1,90,1,.04,5);
  index(60,100);
  forwardPIDLight(1500,-197,3,15);
  index(100,20);
  rollerSpeed(-100);
  task::sleep(300);
  forwardSpeed(20);
  goalScore1();
  backwardPID(780,-197,3,15);
  allSpin(-100);
  task::sleep(450);

  //Goal 5
  leftPID(287,3,1,287,1,.04,5);
  index(60,100);
  forwardPID(1650,-287,3,30);
  leftPID(187,3,1,90,1,.04,5);
  index(60,100);
  forwardPID(800,-187,3,30);
  index(100,30);
  backwardPID(440,-187,3,30);
  leftPID(248,3,1,90,1,.04,5);
  createBallCountTask();
  index(100,30);
  forwardPIDLight(1000,-248,3,30);
  forwardSpeed(20);
  task::sleep(200);
  goalScore();
  stopBallCountTask();
  //rollerSpeed(100);
  conveyorBottom(-100);
  backwardPID(1550,-248,3,30);
  allSpin(-100);
  task::sleep(200);

  //Goal 6
  leftPID(383,3,1,80,.8,.05,5);
  index(60,100);
  forwardPID(1115,-383,3,30);
  index(100,30);
  leftPID(292,3,1,90,1,.04,5);
  forwardPIDLight(1500,-292,3,30);
  task::sleep(100);
  forwardSpeed(20);;
  goalScore1();
  conveyorBottom(-100);
  backwardPID(290,-292,3,30);
  conveyorBottom(-100);

  //goal 7
  leftPID(400,3,1,90,1,.04,5); 
  allSpin(-80);
  task::sleep(250);
  leftPID(380,3,1,90,1,.04,5); 
  index(60,100);
  forwardPID(1300,-380,3,30);
  index(60,100);
  leftPID(393,3,1,90,1,.04,5);
  index(60,100);
  forwardPID(1050,-393,3,30);
  index(60,100);
  backwardPID(670,-393,5,30);
  leftPID(338,3,1,90,1,.04,5);
  index(100,100);
  createBallCountTask();
  forwardPIDLight(1200,-338,3,30);
  forwardSpeed(20);
  task::sleep(200);
  goalScore();
  stopBallCountTask();
  //rollerSpeed(100);
  conveyorBottom(-100);
  backwardPID(650,-338,3,30);
  allSpin(-100);
  task::sleep(400);

  //Goal 8
  leftPID(466,3,1,80,.8,.05,5);
  index(60,100);
  forwardPID(1650,-466,3,30);
  index(60,100);
  task::sleep(400);
  leftPID(376,3,1,90,1,.04,5);
  index(100,30);
  forwardPIDLight(500,-376,3,1);
  task::sleep(100);
  forwardSpeed(20);;
  goalScore1();
  allSpin(-60);
  backwardPID(500,-376,3,30);

  //Goal 9
  leftPID(199.5,3,1,80,.8,.05,5);
  index(100,100);
  rollerSpeed(100);
  forwardPIDLight(1100,-199.5,3,30);
  task::sleep(100);
  forwardSpeed(30);
  task::sleep(500);
  allSpin(100);
  task::sleep(300);
  allSpin(0);
  index(100,100);
  task::sleep(1000);
  allSpin(0);
  backwardPID(1000,-199.5,3,30);






  







  


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
