#pragma once
#include <variant>
#include <vector>
#include <cmath>
#include <numbers>
#include "solar/auto/motion.hpp"
#include "solar/chassis.hpp"

namespace solar {
    inline double average(const std::vector<double>& values) {
        if (values.empty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.size();
    }
    
    inline double degToRad(double deg) {
        return (deg * std::numbers::pi / 180);
    }

    inline double radToDeg(double rad) {
        return (rad * 180 / std::numbers::pi);
    }
    inline int sign(double input) {
        return (input > 0) ? 1 : ((input < 0) ? -1 : 0); // i used 2 ternary operators instead of just doing if elseif else cuz i hate my life
    }
    inline int sign(int input) {
        return (input > 0) ? 1 : ((input < 0) ? -1 : 0); // i used 2 ternary operators instead of just doing if elseif else cuz i hate my life
    }
    inline double getRadius(double x, double y, double x1, double y1, double angle) {
        double deltax = x1 - x;
        double deltay = y1 - y;
        if((2 * deltay * sin(degToRad(90 - angle))) == 0) {
            return 999;
        }
        return (deltax * deltax + deltay * deltay) / (2 * deltay * sin(degToRad(90 - angle)));
    }

    inline double normalizeAngle(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    inline double slew(double target, double prev, double max_change) {
        if (target - prev > max_change) return prev + max_change;
        if (prev - target > max_change) return prev - max_change;
        return target;
    }

    inline double angleError(double target, double current, AngularDirection direction = AngularDirection::AUTO) {
        if (direction == AngularDirection::AUTO) return normalizeAngle(target - current);
        if (direction == AngularDirection::CW)   return std::fmod(target - current + 360, 360);
        return -std::fmod(current - target + 360, 360);
    }

    inline double floorMagnitude(double speed, double min) {
        if (speed < 0 && speed > -min) return -min;
        else if (speed > 0 && speed < min) return min;
        else return speed;
    }

    template <typename A, typename B>
    inline double distance(const A& p1, const B& p2) {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }

    template <typename A, typename B>
    inline double angleto(const A& p1, const B& p2) {
        return radToDeg(std::atan2(p2.x - p1.x, p2.y - p1.y));
    }
}