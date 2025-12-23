#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
pros::MotorGroup leftMotors({-5, -6, -7},
                            pros::MotorGearset::blue); // left motor group - ports 5, 6, 7 (reversed)
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - ports 1, 2, 3
pros::Motor intakeMotor(10, pros::MotorGearset::green);
pros::Motor outtakeMotor(12, pros::MotorGearset::green);

// Inertial Sensor on port 10
pros::Imu imu(10);

//Pneumatics
pros::adi::Pneumatics outtakeP('H', false);
pros::adi::Pneumatics intakeP('G', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                          nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                          nullptr, // horizontal tracking wheel
                          nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                          &imu // inertial sensor 
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors); 

// Movement
void moveForward(float inch, float speed){
	int deg = inch*45;
	rightMotors.move_relative(deg, speed);
	rightMotors.move_relative(-deg, speed);
};

// Turns
void turnR(int degree){
	int deg = degree*8;
	rightMotors.move_relative(-deg, 50);
	rightMotors.move_relative(-deg, 50);
};
void turnL(int degree){
	int deg = degree*8;
	rightMotors.move_relative(deg, 50);
	rightMotors.move_relative(deg, 50);
};

// Output
void output(){
	outtakeMotor.move(127);
	intakeMotor.move(-127);
	pros::delay(2000);
	outtakeMotor.brake();
	intakeMotor.brake();
};

// Intake
void intake(int n){
	if (n==1){
		intakeMotor.move(-127);
	} else if (n==2) {
		intakeMotor.move(127);
	} else {
		pros::delay(2000);
		intakeMotor.brake();
	}
};

// Driver Movement
void driverMove(){
	rightMotors.move(200);
	leftMotors.move(200);

	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){
		rightMotors.move_velocity((controller.get_analog(ANALOG_LEFT_Y)*1.575) - (controller.get_analog(ANALOG_RIGHT_X)*1.575));
		leftMotors.move_velocity((controller.get_analog(ANALOG_LEFT_Y)*1.575) + (controller.get_analog(ANALOG_RIGHT_X)*1.575));
		pros::delay(2);
	};
};

// Driver Intake/Outtake Pistons
void drvierPiston(){
	int outtakePVar = 0.0;
	int intakePVar = 0.0;

	outtakeP.retract();
	intakeP.retract();

	while(true){
	if (controller.get_digital_new_press(DIGITAL_DOWN)) {
      outtakePVar = outtakePVar + 1.0;
      if (outtakePVar == 1.0) {
        OuttakeP.extend();
      }
      else {
        OuttakeP.retract();
        outtakePVar = 0.0;
      }
      waitUntil(!controller.get_digital_new_press(DIGITAL_DOWN));
    } else if (controller.get_digital_new_press(DIGITAL_B)) {
      intakePVar = intakePVar + 1.0;
      if (intakePVar == 1.0) {
        IntakeP.extend();
      }
      else {
        IntakeP.retract();
        intakePVar = 0.0;
      }
      waitUntil(!controller.get_digital_new_press(DIGITAL_B));
    };
	};
};

// Driver Intake/Outtake Motors
float intake = 0.0;
float outtake = 0.0;

void driverScoringM(){
	while (true) {
    if (controller.get_digital_new_press(DIGITAL_R2)) {
      if (intake == 0.0) {
        intake = 1.0;
      }
      else {
        intake = 0.0;
      }
      waitUntil(!controller.get_digital_new_press(DIGITAL_R2));
    } else if (controller.get_digital_new_press(DIGITAL_R1)) {
      if (intake == 0.0) {
        intake = 2.0;
      }
      else {
        intake = 0.0;
      }
      waitUntil(!controller.get_digital_new_press(DIGITAL_R1));
    } else if (controller.get_digital_new_press(DIGITAL_L2)) {
      if (outtake == 0.0) {
        outtake = 1.0;
      }
      else {
        outtake = 0.0;
      }
      waitUntil(!controller.get_digital_new_press(DIGITAL_L2));
    } else if (controller.get_digital_new_press(DIGITAL_L1)) {
      if (outtake == 0.0) {
        outtake = 2.0;
      }
      else {
        outtake = 0.0;
      }
      waitUntil(!controller.get_digital_new_press(DIGITAL_L1));
    };
  };
};

void driverScoringMotors(){
	Intake.spin(forward);
  	Outtake.spin(forward);
		while (true) {
		if (intake == 1.0) {
			intakeMotor.move_velocity(100.0);
    	} else if (intake == 2.0) {
     		 intakeMotor.move_velocity(-100.0);
    	} else if (outtake == 1.0) {
      		outtakeMotor.move_velocity(100.0);
    	} else if (outtake == 2.0) {
      		outtakeMotor.move_velocity(-100.0);
    	} else {
      		intakeMotor.move_velocity(0.0);
      		outtakeMotor.move_velocity(0.0);
    };
  };
};

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
};

void disabled() {}

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

// Right side Autonomous
void autoRight(){
	moveForward(39.5,50);
	pros::delay(1000);
	turnR(70);
	pros::delay(1000);
	intakeP.extend();
	pros::delay(1000);
	intake(1);
	moveForward(24.38,30);
	intake(0);
	pros::delay(1000);
	moveForward(-24,50);
	intakeP.retract();
	pros::delay(1000);
	moveForward(-24,50);
	outtakeP.extend();
	output();
	pros::delay(1000);
	moveForward(14.33,50);
	pros::delay(1000);
	// Right Mid
	turnR(135);
	pros::delay(1000);
	intake(1);
	moveForward(67,30);
	intake(0);
	pros::delay(1000);
	intake(2);
};

// Left side Autonomous
void autoLeft(){
	moveForward(39.5,50);
	pros::delay(1000);
	turnL(70);
	pros::delay(1000);
	intakeP.extend();
	pros::delay(1000);
	intake(1);
	moveForward(24.38,50);
	intake(0);
	pros::delay(1000);
	moveForward(-24,50);
	intakeP.retract();
	pros::delay(1000);
	moveForward(-24,50);
	outtakeP.extend();
	output();
	pros::delay(1000);
	moveForward(14.33,50);
	pros::delay(1000);
	//Left Mid
	turnL(135);
	pros::delay(1000);
	intake(1);
	moveForward(67,30);
	intake(0);
	pros::delay(1000);
	moveForward(-20,50);
	pros::delay(1000);
	turnR(180);
	pros::delay(1000);
	moveForward(-20,50);
	outtakeP.retract();
	output();
};

void autonomous() {
	//chassis.follow(example_txt, inch to look forward, secs how long to run)
	rightMotors.move_relative(39.5,50);
};

//Driver Controls
void opcontrol() {
    pros::Task ws1(driverPiston);
	pros::Task ws2(driverScoringMotors);
	pros::Task ws3(driverScoringM);
	driverMove();
};
