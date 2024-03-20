/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <chrono>
#include <cmath>
#include "knowrob/TimePoint.h"

using namespace knowrob;

TimePoint time::now() {
	auto tp = std::chrono::system_clock::now();
	return std::chrono::time_point_cast<std::chrono::seconds>(tp);
}

TimePoint time::fromSeconds(double seconds) {
	auto tp =
			std::chrono::system_clock::from_time_t(static_cast<time_t>(seconds));
	return std::chrono::time_point_cast<std::chrono::seconds>(tp);
}

double time::toSeconds(const TimePoint &timestamp) {
	auto time_t_value = std::chrono::system_clock::to_time_t(timestamp);
	return static_cast<double>(time_t_value);
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const TimePoint &tp) { // NOLINT
		os << time::toSeconds(tp);
		return os;
	}
}
