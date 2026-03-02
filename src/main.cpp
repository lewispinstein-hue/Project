#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
  mvlib::setOdom(logger, &chassis);
  logger.setRobot({
    .LeftDrivetrain = mvlib::shared(left_mg),
    .RightDrivetrain = mvlib::shared(right_mg)
  });
  chassis.calibrate();
  chassis.setPose(-60, -60, 0);
  logger.start();
}

double expo_joystick_forward(double input, double expoForwards, double deadband) {
  if (fabs(input) < deadband) return 0;
  double normalized = input / 127.0;
  double curved = pow(fabs(normalized), expoForwards);
  curved = curved * (normalized >= 0 ? 1 : -1); // restore sign
  if (fabs(curved) >= 1) return input;
  return curved * 127.0;
}

double expo_joystick_turn(double input, double expoTurn, double deadband = 20) {
  if (fabs(input) < deadband) return 0;
  double norm = input / 127.0;
  double linear = norm;
  double exponential = pow(fabs(norm), expoTurn) * ((norm >= 0) ? 1 : -1);
  double blended = 0.40 * linear + 0.42 * exponential;
  if (fabs(blended) >= 1) return input;
  return blended * 127.0;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

template<class T, class rtn = double>
requires (std::is_arithmetic_v<T> && std::is_arithmetic_v<rtn>)
rtn avg(const std::vector<T>& v) {
  rtn num = 0;
  for (auto& e : v) {
    num += e;
  }
  return v.empty() ? 0 : num /= v.size();
}

void opcontrol() {
  const uint8_t deadband = 10;
  while (true) {
    handleMisc();
    
    double LEFT_Y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double RIGHT_X = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    LEFT_Y = expo_joystick_forward(LEFT_Y, 2.6, deadband);
    RIGHT_X = expo_joystick_turn(RIGHT_X, 1.9, deadband);

    chassis.arcade(LEFT_Y, RIGHT_X);
    pros::delay(20);
  }
}
