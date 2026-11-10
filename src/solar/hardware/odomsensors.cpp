#include "odomsensors.hpp"
#include <cmath>
namespace solar {
    // exit conditions stuff
    OdomSensors::OdomSensors(
        pros::Imu imu,
        TrackingWheel v_tracking_wheel,
        TrackingWheel h_tracking_wheel,
        std::optional<pros::Distance> distance_front,
        std::optional<pros::Distance> distance_left,
        std::optional<pros::Distance> distance_right,
        std::optional<pros::Distance> distance_back
    ):
        imu(imu),
        v_tracking_wheel(v_tracking_wheel),
        h_tracking_wheel(h_tracking_wheel),
        distance_front(distance_front),
        distance_left(distance_left),
        distance_right(distance_right),
        distance_back(distance_back)
    {

    }

    void OdomSensors::calibrateIMU(){
		int attempt = 1;
		while (attempt <= 5) {
			imu.reset();
			do pros::delay(10);
			while (imu.get_status() != pros::ImuStatus::error && imu.is_calibrating());
			if (std::isfinite(imu.get_heading())) {
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
}