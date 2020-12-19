#ifndef AUTONFUNTIONS_H
#define AUTONFUNTIONS_H

void inertialCalibration();
void reset();
void moveForward(int distance, int speed);
void backward(int distance, int speed);
void turnRight(int distance, int speed);
void turnLeft(int distance, int speed);
void intakeRollers(int distance, int speed);
void outtakeRollers(int distance, int speed);
void intakeConveyor(int distance, int speed);
void outtakeConveyor(int distance, int speed);
void forwardTime(int speed);
void backwardTime(int speed);
void forwardPID(int target);
void backwardPID(int target);
void rightPID(int target);
void leftPID(int target);
void forwardSlowPID(int target);
void backwardSlowPID(int target);
void rightSlow(int target);
void leftSlow(int target);
void opticalStopRed();
void opticalStopBlue();
void ballStopRed();
void goalScore(int speed);
void ballStopAll();
void ballTop();
void rollerSpeed(int speed);
void conveyorSpeed(int speed);
void setCoast();
void setHold();
void flipOut(void);
void allSpin(int speed);
void printInet();
#endif
