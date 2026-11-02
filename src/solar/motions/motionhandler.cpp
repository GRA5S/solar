#include "solar/motions/motionhandler.hpp"
#include "solar/motions/pid.hpp"
namespace solar {
    MotionHandler::MotionHandler(
        ControllerSettings linear_controller,
        ControllerSettings angular_controller
    ):
    _linear_controller(linear_controller),
    _angular_controller(angular_controller)
    {
        
    }
}
