#include "solar/hardware/chassis.hpp"
#include "pros/imu.hpp"
#include "trackingwheel.hpp"
#include <cmath>
#include <mutex>

namespace solar {

	Chassis::Chassis(
		std::initializer_list<std::int8_t> left_ports,
		std::initializer_list<std::int8_t> right_ports,
		pros::Imu imu,
		TrackingWheel vwheel,
		TrackingWheel hwheel,
		double wheel_diameter
	): 
		_left_motors(left_ports),
		_right_motors(right_ports),
		_imu(imu),
		_v_tracking_wheel(vwheel),
		_h_tracking_wheel(hwheel),
		_wheel_diameter(wheel_diameter) 
	{
		// body go here idk if we need anythin here
	}

	void Chassis::arcade(std::int32_t throttle, std::int32_t turn) const {
		_left_motors.move(throttle - turn);
		_right_motors.move(throttle + turn);
	}

	void Chassis::stop() const {
		_left_motors.brake();
		_right_motors.brake();
	}

	void Chassis::set_brake_mode(pros::motor_brake_mode_e_t mode) const {
		_left_motors.set_brake_mode_all(mode);
		_right_motors.set_brake_mode_all(mode);
	}

	Pose Chassis::get_pose() const {
		std::lock_guard<pros::Mutex> lock(_pose_mutex);
		return _pose;
	}

	void Chassis::set_pose(Pose pose) {
		{ // curly braces so that at the } the mutex releases
			std::lock_guard<pros::Mutex> lock(_pose_mutex);
			_pose = pose;
		}
		_imu.set_rotation(pose.theta);
	}

	void Chassis::set_pose(double x, double y, double theta) {
		set_pose(Pose{x, y, theta});
	}

	void Chassis::set_pose(double x, double y) {
		double current_theta; // declare so it stays in scope
		{ // curly braces so that at the } the mutex releases
			std::lock_guard<pros::Mutex> lock(_pose_mutex);
			current_theta = _pose.theta;
		}
		set_pose(Pose{x, y, current_theta});
	}

	void Chassis::_update() {
		// TODO: position updating stuff
	}

	void Chassis::_calibrateIMU(){
		int attempt = 1;
		bool calibrated = false;
		while (attempt <= 5) {
			_imu.reset();
			do pros::delay(10);
			while (_imu.get_status() != pros::ImuStatus::error && _imu.is_calibrating());
			if (std::isfinite(_imu.get_heading())) {
				calibrated = true;
				break;
			}
			pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
			printf("IMU failed x%i", attempt);
			attempt++;
		}
		if (attempt > 5) {
			printf("IMU IS COOKED IT DIDNT CALIBRATE D:");
			return;
		}
	}
	
	void Chassis::calibrate() {
		if (_imu.is_installed()) _calibrateIMU();
		_h_tracking_wheel.reset();
		_v_tracking_wheel.reset();
		// rumble to controller to indicate success
		pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
	}
}
