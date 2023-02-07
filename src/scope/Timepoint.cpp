/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <chrono>
#include <cmath>
#include "knowrob/scope/TimePoint.h"

using namespace knowrob;

TimePoint::TimePoint(const double& value)
: value_(value)
{
}

TimePoint TimePoint::now()
{
	auto time = std::chrono::system_clock::now().time_since_epoch();
	std::chrono::seconds seconds = std::chrono::duration_cast< std::chrono::seconds >(time);
	std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(time);
	return { (double) seconds.count() + ((double) (ms.count() % 1000)/1000.0) };
}

bool TimePoint::operator<(const TimePoint& other) const
{
	return value_ < other.value_;
}

bool TimePoint::operator==(const TimePoint& other) const
{
	return fabs(value_ - other.value_) < 1e-9;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const TimePoint& tp) //NOLINT
	{
		return os << tp.value();
	}

	std::ostream& operator<<(std::ostream& os, const Range<TimePoint>& t) //NOLINT
	{
		if(t.min().has_value() && t.max().has_value()) {
			if(t.min().value() == t.max().value()) {
				return os << t.min().value();
			}
			else {
				return os << '(' << t.min().value() << ',' << t.max().value() << ')';
			}
		}
		else if(t.min().has_value()) {
			return os << ">=" << t.max().value();
		}
		else if(t.max().has_value()) {
			return os << "<=" << t.max().value();
		}
		else {
			return os << '*';
		}
	}
}