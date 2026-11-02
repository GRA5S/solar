#pragma once
#include "solar/motions/pid.hpp"
namespace solar {
    enum DriveSide {
		LEFT,
		RIGHT
	};
    class MotionHandler{
        public:
            MotionHandler(
                ControllerSettings linear_contoller,
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
            
        private:
            // motion stuff
			void _requestMotionstart();
			void _endMotion();
			bool _motion_running = false;
			bool _motion_queued = false;
			double dist_travelled = 0;
            // controller settings
			ControllerSettings _linear_controller;
			ControllerSettings _angular_controller;
    };

}