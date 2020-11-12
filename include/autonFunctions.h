#ifndef AUTONFUNTIONS_H
#define AUTONFUNTIONS_H

void inertialCalibration();
void reset();
int iMovePid(int target);
void forwardPID(int target);
void backwardPID(int target);
void moveForward(void);
void gyroTurn(int target);
void rightgyroturn(int h);
#endif
