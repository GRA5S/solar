#pragma once
#include "solar/hardware/odomsensors.hpp"
#include "solar/auto/pid.hpp"
#include "solar/hardware/drivetrain.hpp"
#include <memory>
enum DriveSide {
	LEFT,
	RIGHT
};
struct Pose {
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
};
namespace solar {
	class Chassis{
		public:
			Chassis(
				Drivetrain drivetrain,
				OdomSensors sensors,
				ControllerSettings linear_controller,
				ControllerSettings angular_controller
			);
			// motion stuff
            void waitUntil(double dist);
            void waitUntilDone();
            void cancelMotion();
            void cancelAllMotions();
            bool isInMotion() const;

            // motions
            void turnToPoint(double x, double y, int timeout);
            void turnToHeading(double theta, int timeout);
            void swingToHeading(double theta, DriveSide lockedside, int timeout);
            void swingToPoint(double x, double y, int timeout);
            void moveToPose(double x, double y, double theta, int timeout);
            void moveToPoint(double x, double y, int timeout);

			// pose related funtions
			Pose get_pose() const;
			void set_pose(Pose);
			void set_pose(double x, double y, double theta);
			void set_pose(double x, double y);
			void calibrate();

		private:
			// hardware
			Drivetrain _drivetrain;
			OdomSensors _odom_sensors;

			// motion stuff
			void _requestMotionstart();
			void _endMotion();
			bool _motion_running = false;
			bool _motion_queued = false;
			double _dist_travelled = 0;
			ControllerSettings _linear_controller;
			ControllerSettings _angular_controller;

			// tracking stuff
			Pose _pose{};
			mutable pros::Mutex _pose_mutex;
			void _update();
			double _prev_imu_rad = 0.0;
			std::unique_ptr<pros::Task> _odom_task;
	};

}
