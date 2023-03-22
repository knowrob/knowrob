/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <limits>
#include <cmath>
#include "knowrob/modalities/ConfidenceValue.h"

using namespace knowrob;

ConfidenceValue::ConfidenceValue(double value)
: value_(value)
{
}

const ConfidenceValue& ConfidenceValue::max()
{
	static ConfidenceValue v(0.0);
	return v;
}

const ConfidenceValue& ConfidenceValue::min()
{
	static ConfidenceValue v(1.0);
	return v;
}

bool ConfidenceValue::operator<(const ConfidenceValue& other) const
{
	return value_ < other.value_;
}

bool ConfidenceValue::operator==(const ConfidenceValue& other) const
{
	return this == &other || fabs(value_ - other.value_) < std::numeric_limits<double>::epsilon();
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const ConfidenceValue& confidence) //NOLINT
	{
		return os << confidence.value();
	}
}