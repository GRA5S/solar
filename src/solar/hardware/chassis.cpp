#include "solar/hardware/chassis.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "trackingwheel.hpp"
#include <cmath>
#include <memory>
#include <mutex>
#include <numbers>

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
		// _imu.set_rotation(pose.theta);
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
		
		// tracking wheel stuff
		double deltax = _h_tracking_wheel.readDeltaDistance();
		double deltay = _v_tracking_wheel.readDeltaDistance();
		double xoffset = _h_tracking_wheel.getOffset();
		double yoffset = _v_tracking_wheel.getOffset();
		double prevx, prevy;
		double localx, localy;
		// heading stuff (mostly)
		double headingdeg = _imu.get_rotation();
		double headingrad = headingdeg * std::numbers::pi / 180.0;
		if (!std::isfinite(headingdeg)) return;
		double prevtheta;
		{
			std::lock_guard<pros::Mutex> lock(_pose_mutex);
			prevtheta = _pose.theta * std::numbers::pi / 180.0;
			prevx = _pose.x;
			prevy = _pose.y;
		}
		double deltatheta = headingrad - _prev_imu_rad;
		_prev_imu_rad = headingrad;
		double avgtheta = prevtheta + deltatheta / 2.0;

		// icl i havent read the math too hard i just skimmed it but i implemented this according to the 5225a paper and it seems to be the same as lemlibs sooooo
		const auto localposition = [&] {
			if (deltatheta < 1e-9) {localx = deltax; localy = deltay;} // <1e-9 is basically just checking if its zero cuz floats and doubles are weird
			else {
				/*
				chord length is L=2r*sin(θ/2)

				arc length is s=rθ
				therefore s/θ = r (s represents 𝚫x)
				*/
				localx = 2 * (deltax / deltatheta + xoffset) * std::sin(deltatheta / 2.0);
				localy = 2 * (deltay / deltatheta + yoffset) * std::sin(deltatheta / 2.0);
			}
			/*
			lowk i dont get this part but it translates the position relative to the robot into one relative to the feild
			i could figure it out but ehhh it seems right and unless something gets fucked up ima leave it
			*/
			double dx = localy*std::sin(avgtheta) - localx*std::cos(avgtheta);
			double dy = localy*std::cos(avgtheta) + localx*std::sin(avgtheta);
			{ 
				std::lock_guard<pros::Mutex> lock(_pose_mutex);
				_pose.x = prevx + dx; 
				_pose.y = prevy + dy;
				_pose.theta = (prevtheta + deltatheta) * 180.0 / std::numbers::pi; 
			}
		};



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
		_prev_imu_rad = _imu.get_rotation() * std::numbers::pi / 180.0;
		_h_tracking_wheel.reset();
		_v_tracking_wheel.reset();
		set_pose(0,0,0);
		if (!_odom_task){
			_odom_task = std::make_unique<pros::Task>([this] { // the make unique thing keeps it running after calibrate() ends
				while (true) {_update(); pros::delay(10);}
			});
		}
		pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
	}
}
