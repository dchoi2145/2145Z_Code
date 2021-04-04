#include "vex.h"
#include "autonFunctions.h"
#include "autonSelector.h"

int CURRENT_AUTON = 1;

void printCurrentAuton() {
  Brain.Screen.setCursor(5, 1);

  std::string currentAutonName;
  switch (CURRENT_AUTON) {
    case 0:
      currentAutonName = "Home Row";
      break;
    case 1:
      currentAutonName = "Skills";
      break;
    case 2:
      currentAutonName = "Center";
      break;
  }

  Brain.Screen.print("Current Auton: %s       ", currentAutonName.c_str());
}

void incrementAuton() {
  constexpr int numOfAutons = 3;
  if (CURRENT_AUTON == numOfAutons - 1) {
    CURRENT_AUTON = 0;
  } else {
    CURRENT_AUTON++;
  }
}

std::vector<std::function<void()>> autons {skillsAuton, centerAuton, homerowAuton};

void skillsAuton() {
   // Red Right Side Home row
 //1st
 rollerSpeed(-100);
 task::sleep(300);
 rollerSpeed(100);
 forwardPID(2100,0);
 backwardPID(600,0);
 leftPID(60,1,1);
 forwardPID(650,-60);
 forwardSpeed(30);
 task::sleep(300);
 goalScore(100,3);
 allSpin(-50);
 backwardPID(1000,-60);
 allSpin(0);
 //2nd
 leftPID(220,5,1);
 rollerSpeed(100);
 forwardPID(1690,-220);
 leftPID(112,1,1);
 forwardPID(1200,-112);
 forwardSpeed(10);
 task::sleep(400);
 goalScore(100,2);
 rollerSpeed(-100);
 task::sleep(400);
 allSpin(-40);
 //3rd 
 backwardPID(1200,-112);
 leftPID(170,3,1);
 rollerSpeed(100);
 forwardPID(2300,-170);
 forwardSpeed(30);
 task::sleep(300);
 goalScore(100,2);
 rollerSpeed(100);
 task::sleep(500);
 allSpin(-50);
 backwardPID(1500,-170);
 //4th
 allSpin(0);
 leftPID(287,3,1);
 rollerSpeed(100);
 forwardPID(1620,-287);
 leftPID(198,1,1);
 forwardPID(1000,-198);
 forwardSpeed(30);
 goalScore(100,3);
 allSpin(-400);
 backwardPID(750,-198);
 allSpin(0);
 //5th
 leftPID(288,3,1);
 rollerSpeed(100);
 forwardPID(1800,-288);
 leftPID(234,1,1);
 forwardPID(750,-234);
 forwardSpeed(30);
 task::sleep(200);
 goalScore(100,2);
 rollerSpeed(100);
 task::sleep(500);
 allSpin(-50);
 backwardPID(2000,-234);
 //6th
 leftPID(384,3,1);
 rollerSpeed(100);
 forwardPID(700,-384);
 leftPID(287,3,1);
 forwardPID(1100,-287);
 task::sleep(100);
 forwardSpeed(30);
 task::sleep(200);
 goalScore(100,2);
 allSpin(-40);
 backwardPID(1200,-287);
 allSpin(0);
 //7th
 leftPID(345,3,1);
 rollerSpeed(100);
 forwardPID(2300,-345);
 forwardSpeed(30);
 task::sleep(300);
 goalScore(100,2);
 rollerSpeed(100);
 task::sleep(500);
 backwardPID(400,-345);
 //8th
 leftPID(500,1,1);
 allSpin(-100);
 task::sleep(1000);
 allSpin(0);
 leftPID(463,3,1);
 rollerSpeed(100);
 forwardPID(2050,-463);
 leftPID(374,1,1);
 task::sleep(100);
 forwardSpeed(30);
 task::sleep(300);
 goalScore(100,1);
 allSpin(-60);
 backwardPID(520,-374);
 allSpin(0);
 //9th
 leftPID(197,3,1);
 rollerSpeed(100);
 conveyorSpeed(70);
 forwardPID(1200,-197);
 conveyorSpeed(0);
 rollerSpeed(-100);
 task::sleep(100);
 forwardTime(50,500); 
 backwardTime(30,500);
 task::sleep(100);
 forwardTime(50,500);
 backwardTime(30,500);
 task::sleep(100);
 forwardTime(50,500);
 allSpin(50);
 task::sleep(700);
 backwardPID(1000,-197);
}

void centerAuton() {
  rollerSpeed(-100);
  task::sleep(300);
  forwardPID(1450, 0); 
  rollerSpeed(100);
  task::sleep(200);
  leftPID(61, 3,.7);
  forwardPID(600, -61);
  forwardSpeed(30);
  goalScore(100, 2);
  conveyorSpeed(100);
  task::sleep(300);
  allSpin(0);
  allSpin(-100);
  backwardPID(1400, -61); 
  rightPID(67,5,1);
  rollerSpeed(100);
  forwardPID(1200,67);
  backwardPID(1100,67);
  rightPID(113,5,1);
  forwardPID(1400,113);
  task::sleep(1500);
  allSpin(60);
}

void homerowAuton() {
  rollerSpeed(-100);
  task::sleep(300);
  forwardPID(1430, 0); 
  rollerSpeed(100);
  task::sleep(200);
  leftPID(61, 3,.7); 
  forwardPID(600, -61);
  forwardSpeed(30);
  goalScore(100, 2);
  conveyorSpeed(100);
  task::sleep(300);
  allSpin(0);
  allSpin(-100);
  backwardPID(1200, -61); 
  allSpin(0);
  leftPID(197,3,1);
  allSpin(0);
  task::sleep(200);
  forwardPID(3290, -197);
  leftPID(151,5,1.5);
  task::sleep(200);
  allSpin(100);
  forwardPID(900,-151);
  forwardSpeed(30);
  goalScore(80,2);
  allSpin(100); 
  task::sleep(300);
  allSpin(0);
  backwardPID(1000,-151);
  setCoast();
}
