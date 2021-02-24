#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
//motors
motor FR = motor(PORT15, ratio18_1, true);
motor BR = motor(PORT10, ratio18_1, true);
motor FL = motor(PORT18, ratio18_1, false);
motor BL = motor(PORT20, ratio18_1, false);
motor RightRoller = motor(PORT11, ratio6_1, true);
motor LeftRoller = motor(PORT17, ratio6_1, false);
motor Conveyor1 = motor(PORT19, ratio6_1, true);
motor Conveyor2 = motor(PORT16, ratio6_1, false);
controller Controller1 = controller(primary);

//sensors 
inertial inertial_gyro = inertial(PORT9);
encoder tracker = encoder(Brain.ThreeWirePort.E);
line ballDetector1 = line(Brain.ThreeWirePort.B);
line ballDetector2 = line(Brain.ThreeWirePort.C);
line ballDetector3 = line(Brain.ThreeWirePort.D);
optical optical1 = optical(PORT14);

void vexcodeInit(void) {
  // Nothing to initialize
}