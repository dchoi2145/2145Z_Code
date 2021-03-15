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
void forwardTime(int speed, double t);
void backwardTime(int speed, double t);
void forwardPID(int target);
void backwardPID(int target);
void rightPID(int target, int counterThresh, double accuracy);
void leftPID(int target, int counterThresh, double accuracy);
void forwardSlowPID(int target);
void backwardSlowPID(int target);
void opticalStopRed();
void opticalStopBlue();
void ballStopRed();
void goalScore(int speed);
void ballStopAll();
void rollerSpeed(int speed);
void conveyorSpeed(int speed);
void setCoast();
void setHold();
void flipOut(void);
void allSpin(int speed);
void printInet();
void forwardSpeed(int speed);
void backwardSpeed(int speed);
void rightPID2(int target);
void leftPID2(int target);
void printTracker();
void ballCycle();
void ballCheckMid(int speed, int balls);
void goTo(double finalX, double finalY, int turnDirection, double kP, double kD, 
              double minSpeed, double errorMargin);
#endif
