#include "solar/localization/localizationhandler.hpp"
#include "solar/localization/trackingwheel.hpp"
#include "solar/util.hpp"
#include <cmath>
#include <mutex>
namespace solar {
    LocalizationHandler::LocalizationHandler(
        TrackingWheel v_tracking_wheel,
        TrackingWheel h_tracking_wheel,
        pros::Imu imu
    ) :
    _v_tracking_wheel(v_tracking_wheel),
    _h_tracking_wheel(h_tracking_wheel),
    _imu(imu)
    {

    }
    void LocalizationHandler::_update() {
		
		// tracking wheel stuff
		double deltax = _h_tracking_wheel.readDeltaDistance();
		double deltay = _v_tracking_wheel.readDeltaDistance();
		double xoffset = _h_tracking_wheel.getOffset();
		double yoffset = _v_tracking_wheel.getOffset();
		double prevx, prevy;
		double localx, localy;
		// heading stuff (mostly)
		double headingdeg = _imu.get_rotation();
		double headingrad = degToRad(headingdeg);
		if (!std::isfinite(headingdeg)) return;
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
		
		if (fabs(deltatheta) < 1e-9) {localx = deltax; localy = deltay;} // <1e-9 is basically just checking if its zero cuz floats and doubles are weird
		else {
			/*
			chord length is L=2r*sin(θ/2)

			arc length is s=rθ
			therefore s/θ = r (s represents deltax)
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
			_pose.theta = radToDeg(prevtheta + deltatheta); 
		}
	}

    void LocalizationHandler::_calibrateIMU(){
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

	void LocalizationHandler::calibrate() {
		if (_imu.is_installed()) _calibrateIMU();
		_prev_imu_rad = degToRad(_imu.get_rotation());
		_h_tracking_wheel.reset();
		_v_tracking_wheel.reset();
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
		pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
	}

	Pose LocalizationHandler::get_pose() const {
		std::lock_guard<pros::Mutex> lock(_pose_mutex);
		return _pose;
	}

	void LocalizationHandler::set_pose(Pose pose) {
		{ // curly braces so that at the } the mutex releases	
			std::lock_guard<pros::Mutex> lock(_pose_mutex);
			_pose = pose;
		}
		// _imu.set_rotation(pose.theta);
	}

	void LocalizationHandler::set_pose(double x, double y, double theta) {
		set_pose(Pose{x, y, theta});
	}

	void LocalizationHandler::set_pose(double x, double y) {
		double current_theta; // declare so it stays in scope
		{ // curly braces so that at the } the mutex releases
			std::lock_guard<pros::Mutex> lock(_pose_mutex);
			current_theta = _pose.theta;
		}
		set_pose(Pose{x, y, current_theta});
	}

}