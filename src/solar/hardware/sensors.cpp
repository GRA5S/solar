#include "solar/hardware/sensors.hpp"
#include "trackingwheel.hpp"
#include <cmath>
#include <optional>
#include <cstdio>
namespace solar {
    // exit conditions stuff
    Sensors::Sensors(
        pros::Imu imu,
        TrackingWheel v_tracking_wheel,
        std::optional<TrackingWheel> h_tracking_wheel,
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

    void Sensors::calibrate() {
        if (imu.is_installed()){
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
		v_tracking_wheel.reset();
		if (h_tracking_wheel) {h_tracking_wheel->reset();}
		pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
	}
}