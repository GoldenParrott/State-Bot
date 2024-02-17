#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "api.h"

//Controllers
pros::Controller Master(pros::E_CONTROLLER_MASTER);


//Motors
pros::Motor Indexer(1);

pros::Motor Intake(8);

pros::Motor frontLeft(18,1);
pros::Motor middleLeft(19,1);
pros::Motor backLeft(20,1);
pros::Motor_Group leftWheels({frontLeft, middleLeft, backLeft});

pros::Motor frontRight(13,0);
pros::Motor middleRight(14,0);
pros::Motor backRight(11,0);
pros::Motor_Group rightWheels({frontRight, middleRight, backRight});

pros::Motor_Group allWheels({frontLeft, middleLeft, backLeft, frontRight, middleRight, backRight});


//Pnuematics
pros::ADIDigitalOut plowFrontLeft(1);
pros::ADIDigitalOut plowBackLeft(2);
pros::ADIDigitalOut plowFrontRight(3);
pros::ADIDigitalOut plowBackRight(4);

pros::ADIDigitalOut Shield(5);

pros::ADIDigitalIn ArmLimit(6);

pros::Imu Inertial(18);

//Variables
int drvtrDZ = 10;
int drvtrFB;
int drvtrLR;

int autonnumber;
bool pfl = false;
bool pbl = false;
bool pfr = false;
bool pbr = false;

bool shieldRaised = false;
bool hk = false;
bool rta = false;

#endif