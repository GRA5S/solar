#include "solar/chassis.hpp"
#include "solar/hardware/sensors.hpp"
#include <cmath>
#include <cstdio>

namespace solar {

	Chassis::Chassis(
		Drivetrain drivetrain,
		Sensors& sensors,
		ControllerSettings linear_controller,
		ControllerSettings angular_controller
	): 
		_drivetrain(drivetrain),
		_sensors(sensors),
		_linear_controller(linear_controller),
		_angular_controller(angular_controller)
		
	{
		// body go here idk if we need anythin here
	}
}
