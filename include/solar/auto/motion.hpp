#pragma once

#include "solar/auto/pid.hpp"
#include <optional>
namespace solar {
    enum DriveSide {
        LEFT,
        RIGHT
    };
    enum AngularDirection {
        AUTO,
        CW,
        CCW
    };
    struct MotionParams {
        std::optional<double> maxSpeed = std::nullopt;
        std::optional<double> minSpeed = std::nullopt;
        std::optional<double> earlyExitRange = std::nullopt;
        bool motionChaining = false;
        std::optional<ControllerSettings> controller = std::nullopt;
    };
    struct TurnParams : MotionParams {
        AngularDirection direction = AngularDirection::AUTO;
        std::optional<DriveSide> lockedSide = std::nullopt;
    };
}