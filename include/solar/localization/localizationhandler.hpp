#pragma once
#include <memory>
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "solar/localization/trackingwheel.hpp"
namespace solar {
    struct Pose {
		double x = 0.0;
		double y = 0.0;
		double theta = 0.0;
	};
    class LocalizationHandler {
        public:
			LocalizationHandler (
				TrackingWheel v_tracking_wheel,
				TrackingWheel h_tracking_wheel,
				pros::Imu
			);
            // pose related funtions
			Pose get_pose() const;
			void set_pose(Pose);
			void set_pose(double x, double y, double theta);
			void set_pose(double x, double y);
			void calibrate();
        private:
            // pose stuff
			Pose _pose{};
			mutable pros::Mutex _pose_mutex;
			void _update();
			double _prev_imu_rad = 0.0;

            // odom stuff
			TrackingWheel _v_tracking_wheel;
			TrackingWheel _h_tracking_wheel;
			std::unique_ptr<pros::Task> _odom_task;
			pros::IMU _imu;
			void _calibrateIMU();
    };

}