#include "main.h"

#include "opcontrol/drivetrain.hpp"
#include "opcontrol/mechanisms.hpp"

#pragma once

extern Controller controller;
extern std::shared_ptr<OdomChassisController> chass;
extern std::shared_ptr<SkidSteerModel> thing;
extern Motor intake;
extern MotorGroup hang;
// extern Motor hold;
extern MotorGroup catapult;
extern pros::ADIDigitalOut pistonA;
extern pros::ADIDigitalOut pistonB;