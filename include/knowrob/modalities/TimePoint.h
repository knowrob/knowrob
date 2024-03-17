/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TIME_POINT_H_
#define KNOWROB_TIME_POINT_H_

#include <ostream>

namespace knowrob {
	/**
	 * A point in time.
	 */
	class TimePoint {
	public:
		/**
		 * @param value time in seconds
		 */
		TimePoint(const double& value);

		/**
		 * @return the current system time
		 */
		static TimePoint now();

		/**
		 * @return the value of this time point in seconds
		 */
		const double& value() const { return value_; }

		/**
		 * @param other another time point
		 * @return true if this time point occurs earlier than another one.
		 */
		bool operator<(const TimePoint& other) const;

		/**
		 * @param other another time point
		 * @return true if this time point equals the other in nanosecond resolution.
		 */
		bool operator==(const TimePoint& other) const;

	protected:
		double value_;
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::TimePoint& tp);
}

#endif //KNOWROB_TIME_POINT_H_
