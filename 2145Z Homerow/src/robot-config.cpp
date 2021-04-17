#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
//motors
motor FR = motor(PORT16, ratio6_1, true);
motor BR = motor(PORT11, ratio6_1, false);
motor FL = motor(PORT17, ratio6_1, false);
motor BL = motor(PORT18, ratio6_1, true);
motor RightRoller = motor(PORT1, ratio6_1, false);
motor LeftRoller = motor(PORT19, ratio6_1, true);
motor Conveyor1 = motor(PORT15, ratio6_1, true);//bottom 2
motor Conveyor2 = motor(PORT20, ratio6_1, true);//top 1
controller Controller1 = controller(primary);

//sensors 
inertial inertial_gyro = inertial(PORT12);
encoder tracker = encoder(Brain.ThreeWirePort.C);
encoder tracker2 = encoder(Brain.ThreeWirePort.E);
line ballDetector1 = line(Brain.ThreeWirePort.G);
optical optical1 = optical(PORT14);
line ballDetector2 = line(Brain.ThreeWirePort.H);
bumper bumper1 = bumper(Brain.ThreeWirePort.B);

void vexcodeInit(void) {
  // Nothing to initialize
}