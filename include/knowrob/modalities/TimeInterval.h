/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TIME_INTERVAL_H_
#define KNOWROB_TIME_INTERVAL_H_

#include <memory>
#include <ostream>
#include "FuzzyInterval.h"
#include "TimePoint.h"

namespace knowrob {
	/**
	 * A fuzzy time interval where start and end time point lie within a range.
	 */
	class TimeInterval : public FuzzyInterval<TimePoint> {
	public:
		/**
		 * @param sinceRange the time range where the interval starts
		 * @param untilRange the time range where the interval ends
		 */
		TimeInterval(const Range<TimePoint> &sinceRange, const Range<TimePoint> &untilRange);

		/**
		 * @return a time interval without further constraints on begin and end time point of the interval.
		 */
		static const TimeInterval& anytime();

		/**
		 * @return a time interval that at least intersects with the current time.
		 */
		static TimeInterval currently();

		/**
		 * Intersect this time interval with another one.
		 * @param other another time interval.
		 */
		std::shared_ptr<TimeInterval> intersectWith(const TimeInterval &other) const;
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::TimeInterval& ti);
}

#endif //KNOWROB_TIME_INTERVAL_H_
