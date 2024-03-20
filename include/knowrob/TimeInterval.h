/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TIME_INTERVAL_H_
#define KNOWROB_TIME_INTERVAL_H_

#include <memory>
#include <ostream>
#include <optional>
#include "TimePoint.h"

namespace knowrob {
	/**
	 * A fuzzy time interval where start and end time point lie within a range.
	 */
	class TimeInterval {
	public:
		/**
		 * @param sinceRange the time range where the interval starts
		 * @param untilRange the time range where the interval ends
		 */
		TimeInterval(const std::optional<TimePoint> &since, const std::optional<TimePoint> &until);

		/**
		 * @param other another time interval
		 * @return true if both time intervals are equal
		 */
		bool operator==(const TimeInterval &other) const;

		/**
		 * @return a time interval without further constraints on begin and end time point of the interval.
		 */
		static const TimeInterval &anytime();

		/**
		 * @return a time interval that at least intersects with the current time.
		 */
		static TimeInterval currently();

		/**
		 * @param begin the begin time point of the interval
		 * @param end the end time point of the interval
		 * @return a time interval that is valid between the given time points
		 */
		static TimeInterval during(const TimePoint &begin, const TimePoint &end);

		/**
		 * Intersect this time interval with another one.
		 * @param other another time interval.
		 */
		std::shared_ptr<TimeInterval> intersectWith(const TimeInterval &other) const;

		/**
		 * @return the begin time point of the interval
		 */
		const auto &since() const { return since_; }

		/**
		 * @return the end time point of the interval
		 */
		const auto &until() const { return until_; }

	protected:
		std::optional<TimePoint> since_;
		std::optional<TimePoint> until_;
	};
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::TimeInterval &ti);
}

#endif //KNOWROB_TIME_INTERVAL_H_
