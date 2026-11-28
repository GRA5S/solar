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
    struct MoveParams {
        std::optional<double> maxSpeed = std::nullopt;
        std::optional<double> minSpeed = std::nullopt;
        std::optional<double> earlyExitRange = std::nullopt;
        bool motionChaining = false;
        std::optional<ControllerSettings> linearController = std::nullopt;
        std::optional<ControllerSettings> angularController = std::nullopt;
        bool forwards = true;
        double lead = 0.6;
        double goal_offset = 0.0;
        std::optional<ControllerSettings> lateralController = std::nullopt;
    };
}