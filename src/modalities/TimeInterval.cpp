/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/modalities/TimeInterval.h"

using namespace knowrob;

TimeInterval::TimeInterval(const Range<TimePoint> &sinceRange, const Range<TimePoint> &untilRange)
: FuzzyInterval<TimePoint>(sinceRange, untilRange)
{}

const TimeInterval& TimeInterval::anytime()
{
	// sinceRange: [*,*], untilRange: [*,*]
	static const TimeInterval timeInterval(
			Range<TimePoint>(std::nullopt, std::nullopt),
			Range<TimePoint>(std::nullopt, std::nullopt));
	return timeInterval;
}

TimeInterval TimeInterval::currently()
{
	// sinceRange: [*,Now], untilRange: [Now,*]
	TimePoint now = TimePoint::now();
	return {Range<TimePoint>(std::nullopt, now),
			Range<TimePoint>(now, std::nullopt) };
}

std::shared_ptr<TimeInterval> TimeInterval::intersectWith(const TimeInterval &other) const
{
	return std::make_shared<TimeInterval>(
			minRange_.intersectWith(other.minRange_),
			maxRange_.intersectWith(other.maxRange_));
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const TimeInterval& ti) //NOLINT
	{
		return os << '[' << ti.minRange() << ',' << ti.maxRange() << ']';
	}
}