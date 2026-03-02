#include "globals.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({-12, -1},
                         pros::MotorGearset::blue,
                         pros::v5::MotorUnits::degrees);

pros::MotorGroup right_mg({18, 9},
                          pros::MotorGearset::blue,
                          pros::v5::MotorUnits::degrees);

float ROBOT_WIDTH = 12.25f;
float ROBOT_HEIGHT = 12.40f;

pros::Imu imu(10);

lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              9.1,
                              lemlib::Omniwheel::NEW_4,
                              400,
                              2);

pros::Rotation horizontal(20);
pros::Rotation vertical(8);

lemlib::TrackingWheel vertical1(&vertical, 
                                1.95, 
                                0, -1);

lemlib::TrackingWheel horizontal1(&horizontal,
                                  1.95,
                                  -4);

lemlib::OdomSensors sensors(&vertical1,
                            nullptr,
                            &horizontal1,
                            nullptr,
                            &imu);

lemlib::ControllerSettings lateral_controller(10,
                                             0,
                                             3,
                                             3,
                                             1,
                                             100,
                                             3,
                                             500,
                                             20);

lemlib::ControllerSettings angular_controller(2,
                                              0,
                                              10,
                                              3,
                                              1,
                                              100,
                                              3,
                                              500,
                                              0);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors);

screen::Manager disp;
mvlib::Logger& logger = mvlib::Logger::getInstance();


void handleMisc() {
  typedef enum class ControllerButton {
    BTN_NONE,
    BTN_L1,
    BTN_L2,
    BTN_R1,
    BTN_R2,
    BTN_A,
    BTN_B,
    BTN_X,
    BTN_Y,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
  } ControllerButton;

  struct ButtonBinding {
    ControllerButton action;
    pros::controller_digital_e_t button;
    bool onPress; // true = new_press, false = new_release
  };

  // Map every controller button you care about
  static constexpr ButtonBinding bindings[] = {
      {ControllerButton::BTN_L1,   pros::E_CONTROLLER_DIGITAL_L1,   true},
      {ControllerButton::BTN_R1,   pros::E_CONTROLLER_DIGITAL_R1,   true},
      {ControllerButton::BTN_R2,   pros::E_CONTROLLER_DIGITAL_R2,   true},
      {ControllerButton::BTN_L2,   pros::E_CONTROLLER_DIGITAL_L2,   true},
      {ControllerButton::BTN_B,    pros::E_CONTROLLER_DIGITAL_B,    false},
      {ControllerButton::BTN_A,    pros::E_CONTROLLER_DIGITAL_A,    true},
      {ControllerButton::BTN_Y,    pros::E_CONTROLLER_DIGITAL_Y,    true},
      {ControllerButton::BTN_X,    pros::E_CONTROLLER_DIGITAL_X,    true},
      {ControllerButton::BTN_UP,   pros::E_CONTROLLER_DIGITAL_UP,   true},
      {ControllerButton::BTN_DOWN, pros::E_CONTROLLER_DIGITAL_DOWN, false},
      {ControllerButton::BTN_LEFT, pros::E_CONTROLLER_DIGITAL_LEFT, true},
      {ControllerButton::BTN_RIGHT,pros::E_CONTROLLER_DIGITAL_RIGHT,true},
  };

  ControllerButton event = ControllerButton::BTN_NONE;

  // Find the first button that fired this cycle
  for (const auto &b : bindings) {
    bool triggered = b.onPress
                         ? controller.get_digital_new_press(b.button)
                         : controller.get_digital_new_release(b.button);
    if (triggered) {
      event = b.action;
      break;
    }
  }

  switch (event) {
  case ControllerButton::BTN_L1:
    break;

  case ControllerButton::BTN_R1:
    break;

  case ControllerButton::BTN_R2:
  chassis.setPose(0, 0, 0);
    break;

  case ControllerButton::BTN_L2:
    break;

  case ControllerButton::BTN_B:
    break;

  case ControllerButton::BTN_A:
    break;

  case ControllerButton::BTN_Y:
    break;

  case ControllerButton::BTN_X:
    break;

  case ControllerButton::BTN_DOWN:
    break;

  case ControllerButton::BTN_UP:
    break;

  case ControllerButton::BTN_LEFT:
    break;
  case ControllerButton::BTN_RIGHT:
    break;

  case ControllerButton::BTN_NONE:
  default: break;
  }
}