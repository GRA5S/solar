#pragma once
#include "pros/rotation.hpp"
namespace solar {
	class TrackingWheel {
		public:
			TrackingWheel(
				pros::Rotation encoder,
				double diameter,
				double offset
			);
			void reset();
			double getOffset() const;
			double readDeltaDistance(); // im not supposed to call this getdeltadistance apparently cuz of fucking naming conventions since its not a const
			double getDistance() const;
		private:
			pros::Rotation _encoder;
			double _diameter;
			double _offset;
			double _last_distance = 0;
	};
}