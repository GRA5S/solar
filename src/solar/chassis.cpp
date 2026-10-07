#include "solar/chassis.hpp"
#include "pros/imu.hpp"
#include "solar/util.hpp"
#include <cmath>

namespace solar {

Chassis::Chassis(
	std::initializer_list<std::int8_t> left_ports,
    std::initializer_list<std::int8_t> right_ports,
	pros::Imu imu,
	TrackingWheel vwheel,
	TrackingWheel hwheel,
    double wheel_diameter
): 
	left_motors(left_ports),
    right_motors(right_ports),
	imu(imu),
    v_tracking_wheel(vwheel),
    h_tracking_wheel(hwheel),
	wheel_diameter(wheel_diameter) 
{

// TODO: body here
}

void Chassis::arcade(std::int32_t throttle, std::int32_t turn) const {
	left_motors.move(throttle - turn);
	right_motors.move(throttle + turn);
}

void Chassis::stop() const {
	left_motors.brake();
	right_motors.brake();
}

void Chassis::set_brake_mode(pros::motor_brake_mode_e_t mode) const {
	left_motors.set_brake_mode_all(mode);
	right_motors.set_brake_mode_all(mode);
}

void Chassis::update() {
	// TODO: position updating stuff
}


Pose Chassis::get_pose() const {
	return pose;
}


}
