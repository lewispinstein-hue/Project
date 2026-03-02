#pragma once

// Global hardware and control instances shared across the robot code.

#include "api.h"
#include "lemlib/api.hpp"
#include "mvlib/Optional/logger_optional_lemlib.hpp"
#include "mvlib/core.hpp"
#include "screen.hpp"

extern pros::Controller controller;
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern float ROBOT_WIDTH;
extern float ROBOT_HEIGHT;

extern pros::Imu imu;

extern lemlib::Drivetrain drivetrain;
extern pros::Rotation horizontal;
extern pros::Rotation vertical;
extern lemlib::TrackingWheel horizontal1;
extern lemlib::TrackingWheel vertical1;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;

extern screen::Manager disp;
extern mvlib::Logger& logger;

void handleMisc();