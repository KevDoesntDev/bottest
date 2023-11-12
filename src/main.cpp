#include "main.h"
#include "lemlib/api.hpp"

/**
 * Global Device Initialization
 */
Controller controller(ControllerId::master);
std::shared_ptr<OdomChassisController> chass =
	ChassisControllerBuilder()
		.withMotors({2, 4}, {-1, -3})
		.withGains(
			{0.001, 0.001, 0}, // Distance controller gains
			{0.0033, 0, 0.00013},
			// {0.006, 0.001, 0.0002}, // Turn controller gains
			{0, 0, 0} // Angle controller gains (helps drive straight)
			)
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 9.85_in}, imev5GreenTPR})
		.withOdometry()
		.buildOdometry();
std::shared_ptr<SkidSteerModel> thing = std::dynamic_pointer_cast<SkidSteerModel>(chass->getModel());

// std::shared_ptr<AsyncMotionProfileController> profileController = 
//   AsyncMotionProfileControllerBuilder()
//     .withLimits({
//       1.0, // Maximum linear velocity of the Chassis in m/s
//       2.0, // Maximum linear acceleration of the Chassis in m/s/s
//       10.0 // Maximum linear jerk of the Chassis in m/s/s/s
//     })
//     .withOutput(chassis)
//     .buildMotionProfileController();





pros::Motor left_front_motor(2, true); // port 1, not reversed
pros::Motor left_back_motor(4, true); // port 2, not reversed
pros::Motor right_front_motor(1, false); // port 3, reversed
pros::Motor right_back_motor(3, false); // port 4, reversed
 
// drivetrain motor groups
pros::MotorGroup left_side_motors({left_front_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_back_motor});
 
lemlib::Drivetrain_t drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    9.85, // track width
    4, // wheel diameter
    200 // wheel rpm
};
 
 // odometry struct
lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    nullptr // inertial sensor
};
 
// forward/backward PID
lemlib::ChassisController_t lateralController {
    50, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    0, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};
 
 
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


Motor intake(10, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
MotorGroup hang(
	{Motor(7, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees),
	 Motor(8, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees)});
MotorGroup catapult({Motor(6, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees), Motor(9, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees)});
pros::ADIDigitalOut pistonA('A');
pros::ADIDigitalOut pistonB('H');
// MotorGroup lift(
// 	{Motor(5, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees),
// 	 Motor(8, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees),
// 	 Motor(20, false, AbstractMotor::gearset::green,		 AbstractMotor::encoderUnits::degrees)});
// Motor arm(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// pros::ADIDigitalOut piston('A');

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}


void initialize()
{
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::Task screenTask(screen); // create a task to print the position to the screen

	catapult.setBrakeMode(AbstractMotor::brakeMode::hold);
	hang.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::coast);
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

void autonomous()
{
	chassis.moveTo(10, 0, 1000);

	// chassis->setState({0_in, 0_in, 0_deg});

	// // chassis->moveDistance(1_ft);

	// intake.moveVoltage(-12000);
	// chassis->moveDistance(2_ft);
	// chassis->moveDistance(-2_ft);

	// chassis->turnAngle(-45_deg);
	// chassis->moveDistance(-22_in);
	// pistonA.set_value(true);
	// chassis->turnAngle(-45_deg);
	// chassis->moveDistance(-20_in);

	// chassis->moveDistance(12_in);
	// chassis->turnAngle(-180_deg);
	// intake.moveVoltage(12000);
	// chassis->moveDistance(12_in);

	// chassis->turnAngle(-90_deg);



	//drivetrain->driveVectorVoltage(12000, 0);

	// intake.moveVelocity(-600);
	// drivetrain->driveVectorVoltage(-4000, 0);
	// pros::delay(500);
	// drivetrain->stop();
	// chassis->setMaxVelocity(140);
	// chassis->moveDistance(3.9_ft);
	// chassis->setMaxVelocity(75);
	// chassis->turnAngle(100_deg);
	// drivetrain->driveVectorVoltage(1000, 0);
	// pros::delay(1000);
	// drivetrain->stop();
	// intake.moveVelocity(600);
	// pros::delay(100);
	// intake.moveVelocity(0);
	// chassis->setMaxVelocity(140);
	// chassis->moveDistance(-6_in);
	// intake.moveVelocity(-600);
	// chassis->setMaxVelocity(75);
	// chassis->turnAngle(-235_deg);
	// chassis->setMaxVelocity(140);
	// chassis->moveDistance(1.8_ft);
	// chassis->setMaxVelocity(75);
	// chassis->turnAngle(195_deg);
	// intake.moveVelocity(0);
	// piston.set_value(true);
	// chassis->setMaxVelocity(75);
	// chassis->turnAngle(15_deg);
	// drivetrain->driveVectorVoltage(10000, 0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	// chassis->stop();
	pros::Task drivetrainTask(DrivetrainLoop, (void *)"", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "drivetrain");

	pros::Task mechanismsTask(MechanismsLoop, (void *)"", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "mechanisms");

	while (true)
	{
		pros::delay(20);
	}
}
