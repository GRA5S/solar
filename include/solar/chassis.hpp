#pragma once
#include <cstdint>
#include <initializer_list>
#include <memory>
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

namespace solar {

struct Pose {
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
};
class TrackingWheel {
	public:
		TrackingWheel(
			pros::Rotation encoder,
			double diameter,
			double offset
		);
		void reset();
		double getOffset() const;
		double readDeltaDistance();
		double getDistance() const;
	private:
		pros::Rotation encoder;
		double diameter;
		double offset;
		double last_distance;
};

class Chassis {
	public:
		Chassis(
			std::initializer_list<std::int8_t> left_ports,
			std::initializer_list<std::int8_t> right_ports,
			pros::Imu imu,
			TrackingWheel v_tracking_wheel,
			TrackingWheel h_tracking_wheel,
		    double wheel_diameter = 3.25
		);
		void calibrate();

		// drive related functions
		void arcade(std::int32_t throttle, std::int32_t turn) const;
		void stop() const;
		void set_brake_mode(pros::motor_brake_mode_e_t mode) const;

		// pose related funtions
		Pose get_pose() const;
		void set_pose(Pose);
		void set_pose(double x, double y, double theta);
		void set_pose(double x, double y);

	private:
		// drive stuff
		pros::IMU imu;
		TrackingWheel v_tracking_wheel;
		TrackingWheel h_tracking_wheel;
		pros::MotorGroup left_motors;
		pros::MotorGroup right_motors;
		double wheel_diameter;

		// pose stuff
		Pose pose{};
		void update();

		// odom stuff
		double prev_left_rot = 0.0;
		double prev_right_rot = 0.0;
		std::unique_ptr<pros::Task> odom_task;
};

}
