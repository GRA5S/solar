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
    struct TurnParams {
        std::optional<double> maxSpeed = std::nullopt;
        std::optional<double> minSpeed = std::nullopt;
        std::optional<double> earlyExitRange = std::nullopt;
        bool motionChaining = false;
        std::optional<ControllerSettings> controller = std::nullopt;
        AngularDirection direction = AngularDirection::AUTO;
        std::optional<DriveSide> lockedSide = std::nullopt;
        double leftMultiplier = 1;
        double rightMultiplier = 1;
    };
}