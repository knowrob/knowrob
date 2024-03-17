/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/modalities/TimeInterval.h"

using namespace knowrob;

TimeInterval::TimeInterval(const std::optional<TimePoint> &since, const std::optional<TimePoint> &until)
		: since_(since),
		  until_(until) {}

bool TimeInterval::operator==(const TimeInterval &other) const {
	return since_ == other.since_ && until_ == other.until_;
}

const TimeInterval &TimeInterval::anytime() {
	static const TimeInterval timeInterval(std::nullopt, std::nullopt);
	return timeInterval;
}

TimeInterval TimeInterval::currently() {
	TimePoint now = time::now();
	return {now, now};
}

TimeInterval TimeInterval::during(const TimePoint &begin, const TimePoint &end) {
	return {begin, end};
}

std::shared_ptr<TimeInterval> TimeInterval::intersectWith(const TimeInterval &other) const {
	return std::make_shared<TimeInterval>(std::max(since_, other.since_), std::min(until_, other.until_));
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const TimeInterval &ti) { //NOLINT
		os << '[';
		if (ti.since().has_value()) os << ti.since().value();
		else os << '_';
		os << ",";
		if (ti.until().has_value()) os << ti.until().value();
		else os << '_';
		os << ']';
		return os;
	}
}
