#include "solar/chassis.hpp"
#include "pros/rtos.hpp"
#include "solar/hardware/sensors.hpp"
#include "util.hpp"
#include <cmath>
#include <cstdio>

namespace solar {
	Chassis::Chassis(
		Drivetrain drivetrain,
		Sensors& sensors,
		ControllerSettings& linear_controller,
		ControllerSettings& angular_controller
	): 
		_drivetrain(drivetrain),
		_sensors(sensors),
		_linear_controller(linear_controller),
		_angular_controller(angular_controller),
		_linear_pid(_linear_controller),
		_angular_pid(_angular_controller) // make sure to reference the actual member instead of the thingy passed to the constructer because that would go out of scope as soon as the constructor finishes	
	{ }
	void Chassis::init() {
		_sensors.calibrate();
		_prev_imu_rad = degToRad(_sensors.imu.get_rotation());
		set_pose(0,0,0);
		if (!_odom_task){
			_odom_task = std::make_unique<pros::Task>([this] { // the make unique thing keeps it running after calibrate() ends
				uint32_t prev = pros::millis();
				while (true) {
					_update();
					// basically this just makes sure there is atleast 10ms between updates. mainly just doin this cuz i js realized lemlib does it
					if (pros::millis() - prev > 10) prev = pros::millis();
					pros::Task::delay_until(&prev, 10);
				}
			});
		}
		if (!_motion_task){
			_motion_task = std::make_unique<pros::Task>([this]{
				_motionLoop();
			});
		}
	}
}
