#include "solar/hardware/trackingwheel.hpp"
#include "pros/rotation.hpp"
#include <numbers>

namespace solar {
    TrackingWheel::TrackingWheel(
        pros::Rotation encoder,
        double diameter,
        double offset
    ):
    _encoder(encoder),
    _diameter(diameter),
    _offset(offset)
    {
        TrackingWheel::reset();
    }
    double TrackingWheel::readDeltaDistance() {
        double current = getDistance();
        double delta = current - _last_distance;
        _last_distance = current;
        return delta;
    }
    double TrackingWheel::getOffset() const {
        return _offset;
    }
    void TrackingWheel::reset() {
        _encoder.reset_position();
        _last_distance = 0;
    }
    double TrackingWheel::getDistance() const{
        return _encoder.get_position() /100.0 / 360.0 * std::numbers::pi * _diameter;
    }
}