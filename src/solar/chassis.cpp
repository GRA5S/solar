#include "solar/chassis.hpp"
#include "solar/hardware/odomsensors.hpp"
#include <cmath>
#include <cstdio>

namespace solar {

	Chassis::Chassis(
		Drivetrain drivetrain,
		OdomSensors odom_sensors,
		ControllerSettings linear_controller,
		ControllerSettings angular_controller
	): 
		_drivetrain(drivetrain),
		_odom_sensors(odom_sensors),
		_linear_controller(linear_controller),
		_angular_controller(angular_controller)
		
	{
		// body go here idk if we need anythin here
	}
}
