#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
//motors
motor FR = motor(PORT15, ratio18_1, true);
motor BR = motor(PORT10, ratio18_1, true);
motor FL = motor(PORT18, ratio18_1, true);
motor BL = motor(PORT20, ratio18_1, true);
motor RightRoller = motor(PORT11, ratio6_1, false);
motor LeftRoller = motor(PORT17, ratio6_1, true);
motor Conveyor1 = motor(PORT19, ratio6_1, true);
motor Conveyor2 = motor(PORT16, ratio6_1, false);
controller Controller1 = controller(primary);

//sensors
inertial inertial_gyro = inertial(PORT5);
line ballDetector1 = line(Brain.ThreeWirePort.H);
line ballDetector2 = line(Brain.ThreeWirePort.A);

void vexcodeInit(void) {
  // Nothing to initialize
}