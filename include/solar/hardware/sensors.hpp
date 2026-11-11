#pragma once
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "solar/hardware/trackingwheel.hpp"
#include <optional>
namespace solar {
    class Sensors{
        public:
            Sensors(
                pros::Imu imu,
                TrackingWheel v_tracking_wheel,
                std::optional<TrackingWheel> h_tracking_wheel = std::nullopt,
                std::optional<pros::Distance> distance_front = std::nullopt,
                std::optional<pros::Distance> distance_left = std::nullopt,
                std::optional<pros::Distance> distance_right = std::nullopt,
                std::optional<pros::Distance> distance_back = std::nullopt
            );
            pros::IMU imu;
			void calibrate();
            TrackingWheel v_tracking_wheel;
			std::optional<TrackingWheel> h_tracking_wheel;
            std::optional<pros::Distance> distance_front;
            std::optional<pros::Distance> distance_left;
            std::optional<pros::Distance> distance_right;
            std::optional<pros::Distance> distance_back;
    };
}