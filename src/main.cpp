#include "main.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "solar/auto/motion.hpp"
#include "solar/chassis.hpp"
#include "solar/hardware/drivetrain.hpp"
#include "solar/hardware/sensors.hpp"
#include "solar/hardware/trackingwheel.hpp"
#include "solar/util.hpp"

const double DIST_FROM_GOAL = 7.5;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({7, 10, -8}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-4, 3, -2}, pros::MotorGearset::blue);
pros::MotorGroup sidetoggle({-5}, pros::MotorGearset::green);
pros::MotorGroup claw({-16}, pros::MotorGearset::green);
pros::MotorGroup btoggle({13}, pros::MotorGearset::green);

pros::MotorGroup liftMotors({9, -1}, pros::MotorGearset::green); // lift motor group - ports 4, 2 (reversed)

// claw piston, ADI port H
pros::adi::Pneumatics rsalute('g', true);
pros::adi::Pneumatics lsalute('h', true);

pros::Imu imu(14);

// tracking wheels
pros::Rotation horizontal_encoder(15);
pros::Rotation vertical_encoder(6);

solar::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	2.75
);

solar::Sensors sensors(
	imu,
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
            pros::delay(100);
        }
    });
}
void goalReset(double goalx, double goaly) {
    chassis.waitUntilDone();
    double theta = solar::degToRad(chassis.get_pose().theta);

    double x = goalx - DIST_FROM_GOAL * sin(theta);
    double y = goaly - DIST_FROM_GOAL * cos(theta);

    chassis.set_pose(x, y, chassis.get_pose().theta);
}
void autonomous() {
	chassis.set_pose(-60, -34, 49);
	drivetrain.tank(32, 32); pros::delay(700);
	chassis.moveToPoint(-56, -41, 1000, {.minSpeed = 32, .forwards = false});
	chassis.turnToPoint(-47, -47, 700, {.minSpeed = 32});
	chassis.moveToPoint(-47, -47, 1000, {.goal_offset = DIST_FROM_GOAL});
	chassis.turnToPoint(-47, -23.5, 1000);
	chassis.moveToPoint(-47, -23.5, 1000, {.goal_offset = DIST_FROM_GOAL});
	chassis.waitUntilDone();drivetrain.tank(32,32); pros::delay(900); goalReset(-47, -23.5);
	chassis.moveToPoint(-47, -47, 1000, {.forwards = false});
	chassis.turnToPoint(-23.5, -23.5, 1000);
	chassis.moveToPoint(-23.5, -23.5, 1000, {.goal_offset = DIST_FROM_GOAL});
	chassis.turnToPoint(-47, 23.5, 1000);
	chassis.moveToPoint(-47, 23.5, 1000, {.goal_offset = DIST_FROM_GOAL});
	chassis.waitUntilDone();drivetrain.tank(32,32); pros::delay(900); goalReset(-47, -23.5);

}

void opcontrol() {
	chassis.cancelAllMotions();
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    pros::Task mechanismTask([]() {
        int printCounter = 0;
        double liftJogVoltage = 0;
        const double liftJogSlewRate = 500;
        const double liftJogKickVoltage = 5000; // initial jump so the lift doesn't crawl before ramping up
        while (true) {

            double liftJogTarget = 0;
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftJogTarget = 6000;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftJogTarget = -12000;
            }

            static bool wasJogging = false;
            bool isJogging = (liftJogTarget != 0);
            if (isJogging) {
                if (!wasJogging) {
                    liftJogVoltage = std::clamp(liftJogTarget, -liftJogKickVoltage, liftJogKickVoltage);
                }
                if (liftJogVoltage < liftJogTarget) {
                    liftJogVoltage = std::min(liftJogVoltage + liftJogSlewRate, liftJogTarget);
                } else if (liftJogVoltage > liftJogTarget) {
                    liftJogVoltage = std::max(liftJogVoltage - liftJogSlewRate, liftJogTarget);
                }
                liftMotors.move_voltage((int) liftJogVoltage);
            } else if (wasJogging) {
                liftMotors.brake();
                liftJogVoltage = 0;
            }
            wasJogging = isJogging;


if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
                rsalute.set_value(true);
            } else {
                rsalute.set_value(false);
            }

         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                lsalute.set_value(false);
            } else {
                lsalute.set_value(true);
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                btoggle.move_voltage(-12000);
            } else {
                btoggle.brake();
                btoggle.move_voltage(0);
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                claw.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                claw.move_voltage(-10000);
            }
            else {
                claw.brake();
                claw.move_voltage(0);
            }

            if (printCounter % 50 == 0) {
				int liftHottest = (int) std::max(liftMotors.get_temperature(0), liftMotors.get_temperature(1));
				double driveHottestD = leftMotors.get_temperature(0);
				driveHottestD = std::max(driveHottestD, leftMotors.get_temperature(1));
				driveHottestD = std::max(driveHottestD, leftMotors.get_temperature(2));
				driveHottestD = std::max(driveHottestD, rightMotors.get_temperature(0));
				driveHottestD = std::max(driveHottestD, rightMotors.get_temperature(1));
				driveHottestD = std::max(driveHottestD, rightMotors.get_temperature(2));
				int driveHottest = (int) driveHottestD;
				controller.print(0, 0, "L:%dC,D:%dC", liftHottest, driveHottest);
            }
            printCounter++;

            pros::delay(10);
        }
    });

    // drivetrain loop — runs independently at a steady 10ms cadence
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        drivetrain.arcade(leftY, rightX);
        pros::delay(10);
    }
}