#pragma once
#include "pros/rtos.hpp"
#include "solar/hardware/sensors.hpp"
#include "solar/auto/pid.hpp"
#include "solar/hardware/drivetrain.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <queue>
namespace solar {
	enum DriveSide {
		LEFT,
		RIGHT
	};
	struct Pose {
		double x = 0.0;
		double y = 0.0;
		double theta = 0.0;
	};
	class Chassis{
		public:
			Chassis(
				Drivetrain drivetrain,
				Sensors& sensors,
				ControllerSettings& linear_controller,
				ControllerSettings& angular_controller
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
            void swingToPoint(double x, double y, DriveSide lockedside, int timeout);
            void moveToPose(double x, double y, double theta, int timeout);
            void moveToPoint(double x, double y, int timeout);

			// pose related funtions
			Pose get_pose() const;
			void set_pose(Pose);
			void set_pose(double x, double y, double theta);
			void set_pose(double x, double y);

			void init();

		private:
			// hardware
			Drivetrain _drivetrain;
			Sensors& _sensors;

			// motion stuff 
			std::queue<std::function<void()>> _motion_queue;
			mutable pros::Mutex _motion_mutex;
			std::unique_ptr<pros::Task> _motion_task;
			std::optional<double> _motion_progress = std::nullopt;
			bool _motion_running = false;
			void _newMotion(std::function<void()> motion);
			void _motionLoop();

			// pid controller stuff
			ControllerSettings& _linear_controller;
			ControllerSettings& _angular_controller;
			PID _linear_pid;
			PID _angular_pid;

			// tracking stuff
			Pose _pose{};
			mutable pros::Mutex _pose_mutex;
			void _update();
			double _prev_imu_rad = 0.0;
			std::unique_ptr<pros::Task> _odom_task;
	};

}
