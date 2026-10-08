#pragma once
#include <cstdint>
#include <initializer_list>
#include <memory>
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "trackingwheel.hpp"
namespace solar {

	struct Pose {
		double x = 0.0;
		double y = 0.0;
		double theta = 0.0;
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
			pros::IMU _imu;
			pros::MotorGroup _left_motors;
			pros::MotorGroup _right_motors;
			double _wheel_diameter;

			// pose stuff
			Pose _pose{};
			mutable pros::Mutex _pose_mutex;
			void _update();
			double _prev_imu_rad = 0.0;

			// odom stuff
			TrackingWheel _v_tracking_wheel;
			TrackingWheel _h_tracking_wheel;
			std::unique_ptr<pros::Task> _odom_task;
			void _calibrateIMU();
	};

}
