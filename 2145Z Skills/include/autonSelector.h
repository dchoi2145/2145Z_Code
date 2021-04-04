#pragma once

#include "vex.h"
#include <vector>

extern int CURRENT_AUTON;

void printCurrentAuton(void);
void incrementAuton(void);

extern std::vector<std::function<void()>> autons;

void skillsAuton(void);
void centerAuton(void);
void homerowAuton(void);
