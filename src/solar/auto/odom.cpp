#include "solar/chassis.hpp"
#include "solar/hardware/sensors.hpp"
#include "solar/hardware/trackingwheel.hpp"
#include "solar/util.hpp"
#include <mutex>
#include <optional>
namespace solar {
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
	}
    void Chassis::_update() {
		// heading stuff (mostly)
		double headingdeg = _sensors.imu.get_rotation();
		if (!std::isfinite(headingdeg)) return;
		double headingrad = degToRad(headingdeg);

        // tracking wheel stuff
		double deltay = _sensors.v_tracking_wheel.readDeltaDistance();
		double yoffset = _sensors.v_tracking_wheel.getOffset();
		double deltax, xoffset;
		if(_sensors.h_tracking_wheel){
			deltax = _sensors.h_tracking_wheel->readDeltaDistance();
			xoffset = _sensors.h_tracking_wheel->getOffset();
		}

		double prevx, prevy;
		double localx, localy;
		double prevtheta;

		{
			std::lock_guard<pros::Mutex> lock(_pose_mutex);
			prevtheta = degToRad(_pose.theta);
			prevx = _pose.x;
			prevy = _pose.y;
		}

		double deltatheta = headingrad - _prev_imu_rad;
		_prev_imu_rad = headingrad;
		double avgtheta = prevtheta + deltatheta / 2.0;

		// icl i havent read the math too hard i just skimmed it but i implemented the following according to the 5225a paper and it seems to be the same as lemlibs sooooo
		if (!_sensors.h_tracking_wheel) {localx = 0;} 
		else if (fabs(deltatheta) < 1e-9){localx = deltax;} // <1e-9 is basically just checking if its zero cuz floats and doubles are weird
		if (fabs(deltatheta) < 1e-9) {localy = deltay;} 
		else {
			/*
			chord length is L=2r*sin(θ/2)

			arc length is s=rθ
			therefore s/θ = r (s represents deltax)
			*/
			if (_sensors.h_tracking_wheel){
				localx = 2 * (deltax / deltatheta + xoffset) * std::sin(deltatheta / 2.0);
			}
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
			_pose.theta = radToDeg(prevtheta + deltatheta); 
		}
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

}