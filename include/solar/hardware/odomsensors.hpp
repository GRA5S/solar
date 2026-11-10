#pragma once
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "solar/hardware/trackingwheel.hpp"
#include <optional>
namespace solar {
    class OdomSensors{
        public:
            OdomSensors(
                pros::Imu imu,
                TrackingWheel v_tracking_wheel,
                TrackingWheel h_tracking_wheel,
                std::optional<pros::Distance> distance_front,
                std::optional<pros::Distance> distance_left,
                std::optional<pros::Distance> distance_right,
                std::optional<pros::Distance> distance_back
            );
            pros::IMU imu;
			void calibrateIMU();
            TrackingWheel v_tracking_wheel;
			TrackingWheel h_tracking_wheel;
            std::optional<pros::Distance> distance_front;
            std::optional<pros::Distance> distance_left;
            std::optional<pros::Distance> distance_right;
            std::optional<pros::Distance> distance_back;
    };
}