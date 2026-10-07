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
			double readDeltaDistance();
			double getDistance() const;
		private:
			pros::Rotation _encoder;
			double _diameter;
			double _offset;
			double _last_distance = 0;
	};
}