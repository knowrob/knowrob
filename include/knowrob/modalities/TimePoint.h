/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TIME_POINT_H_
#define KNOWROB_TIME_POINT_H_

#include <ostream>
#include <chrono>

namespace knowrob {
	/**
	 * A time point in seconds.
	 */
	typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::seconds> TimePoint;

	namespace time {
		/**
		 * @return the current time point.
		 */
		TimePoint now();

		/**
		 * @param seconds the number of seconds since the epoch.
		 * @return the time point.
		 */
		TimePoint fromSeconds(double seconds);

		/**
		 * @param timestamp the time point.
		 * @return the number of seconds since the epoch.
		 */
		double toSeconds(const TimePoint &timestamp);
	} // namespace time
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::TimePoint &tp);
}

#endif //KNOWROB_TIME_POINT_H_
