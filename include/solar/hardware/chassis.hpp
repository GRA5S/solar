#pragma once
#include <cstdint>
#include <initializer_list>
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "solar/localization/trackingwheel.hpp"
#include "solar/motions/pid.hpp"
#include "solar/motions/motionhandler.hpp"
#include "solar/localization/localizationhandler.hpp"
namespace solar {
	class Chassis : public MotionHandler, public LocalizationHandler{
		public:
			Chassis(
				std::initializer_list<std::int8_t> left_ports,
				std::initializer_list<std::int8_t> right_ports,
				pros::Imu imu,
				TrackingWheel v_tracking_wheel,
				TrackingWheel h_tracking_wheel,
				double wheel_diameter,
				ControllerSettings linear_controller,
				ControllerSettings angular_controller
			);
			// drive related functions
			void arcade(std::int32_t throttle, std::int32_t turn) const;
			void tank(int left, int right) const;
			void stop() const;
			void set_brake_mode(pros::motor_brake_mode_e_t mode) const;
		private:
			// drive stuff
			pros::MotorGroup _left_motors;
			pros::MotorGroup _right_motors;
			double _wheel_diameter;
	};

}
