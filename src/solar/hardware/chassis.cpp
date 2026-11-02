#include "solar/hardware/chassis.hpp"
#include "pros/imu.hpp"
#include "solar/localization/trackingwheel.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace solar {

	Chassis::Chassis(
		std::initializer_list<std::int8_t> left_ports,
		std::initializer_list<std::int8_t> right_ports,
		pros::Imu imu,
		TrackingWheel v_tracking_wheel,
		TrackingWheel h_tracking_wheel,
		double wheel_diameter,
		ControllerSettings linear_controller,
		ControllerSettings angular_controller
	): 
		MotionHandler(linear_controller, angular_controller),
		LocalizationHandler(v_tracking_wheel, h_tracking_wheel, imu),
		_left_motors(left_ports),
		_right_motors(right_ports),
		_wheel_diameter(wheel_diameter)
	{
		// body go here idk if we need anythin here
	}

	void Chassis::arcade(std::int32_t throttle, std::int32_t turn) const {
		_left_motors.move(throttle - turn);
		_right_motors.move(throttle + turn);
	}

	void Chassis::tank(int left, int right) const {
		_left_motors.move(left);
        _right_motors.move(right);
	};

	void Chassis::stop() const {
		_left_motors.brake();
		_right_motors.brake();
	}

	void Chassis::set_brake_mode(pros::motor_brake_mode_e_t mode) const {
		_left_motors.set_brake_mode_all(mode);
		_right_motors.set_brake_mode_all(mode);
	}
}
