#ifndef AUTONFUNTIONS_H
#define AUTONFUNTIONS_H

void inertialCalibration();
void reset();
void forwardTime(double speed, double t);
void backwardTime(double speed, double t);
void forwardSpeed(double speed);
void backwardSpeed(double speed);
void forwardPID(double target, double headingVal, double counterThresh, double accuracy);
void backwardPID(double target, double headingVal, double counterThresh, double accuracy);
void rightPID(double target, double counterThresh, double accuracy, double maxSpeed, double kP, double kI, double kD);
void leftPID(double target, double counterThresh, double accuracy, double maxSpeed, double kP, double kI, double kD);
void forwardSlowPID(double target);
void backwardSlowPID(double target);
void goalScore(double speed, double balls, double time);
void rollerSpeed(double speed);
void conveyorBottom(double speed);
void conveyorTop(double speed);
void setCoast();
void setHold();
void flipOut(void);
void allSpin(double speed);
void prdoubleInet();
void rightPID2(double target);
void leftPID2(double target);
void prdoubleTracker();
void ballCycle();
void ballCheckMid(double speed, double balls);
void ballCycleLine();
void goalScoreLine(double speed, double balls);
void index(double speed, double rollers);
void spit(double speedRight, double speedLeft);
void forwardPIDLight(double target, double headingVal, double counterThresh, double accuracy);
void goalScore1();
void leftTurn(double target, double counterThresh, double accuracy, double maxSpeed, double Speed);
void goalScoree(double speed, double balls, double time);
void goalScoreee(double speed, double balls, double time);
void goalScoreeee(double speed, double balls, double time);

#endif
