#include "main.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "solar/chassis.hpp"
#include "solar/hardware/drivetrain.hpp"
#include "solar/hardware/sensors.hpp"
#include "solar/hardware/trackingwheel.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({1, 2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue); 
pros::Rotation vertical_encoder(8);
pros::Rotation horizontal_encoder(9);

solar::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	2.75
);

solar::Sensors sensors(
	pros::Imu(7),
	solar::TrackingWheel(
		&vertical_encoder,
		2.75,
		0
	),
	solar::TrackingWheel(
		&horizontal_encoder,
		2.00,
		0
	)
);

solar::ControllerSettings linear_settings{
    .kp = 8,
    .kd = 58
};
solar::ControllerSettings angular_settings{
    .kp = 2,
    .kd = 13.875
};

solar::Chassis chassis(
	drivetrain,
	sensors,
	linear_settings,
	angular_settings
);

void initialize() {
	drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.init();
    pros::Task loggingTask([&]() {
        while (true) {
            printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
                   chassis.get_pose().x, chassis.get_pose().y, chassis.get_pose().theta, pros::millis());
            pros::delay(50);
        }
    });
}

void autonomous() {}

void opcontrol() {
	chassis.cancelAllMotions();
	while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        drivetrain.arcade(leftY, rightX);
        pros::delay(10);
    }
}