/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/modalities/ConfidenceInterval.h"

using namespace knowrob;

ConfidenceInterval::ConfidenceInterval(const Range<ConfidenceValue> &minRange,
									   const Range<ConfidenceValue> &maxRange)
: FuzzyInterval<ConfidenceValue>(minRange,maxRange)
{
}

const ConfidenceInterval& ConfidenceInterval::any()
{
	// minRange: [*,*], maxRange: [*,*]
	static const ConfidenceInterval interval(
			Range<ConfidenceValue>(std::nullopt, std::nullopt),
			Range<ConfidenceValue>(std::nullopt, std::nullopt));
	return interval;
}

const ConfidenceInterval& ConfidenceInterval::certain()
{
	// minRange: [MAX_CONFIDENCE,MAX_CONFIDENCE], maxRange: [MAX_CONFIDENCE,MAX_CONFIDENCE]
	static const ConfidenceInterval interval(
			Range<ConfidenceValue>(ConfidenceValue::max(), ConfidenceValue::max()),
			Range<ConfidenceValue>(ConfidenceValue::max(), ConfidenceValue::max()));
	return interval;
}

ConfidenceInterval ConfidenceInterval::at_least(const ConfidenceValue &value)
{
	// minRange: [$VALUE,*], maxRange: [*,*]
	return { Range<ConfidenceValue>(value, std::nullopt),
			 Range<ConfidenceValue>(std::nullopt, std::nullopt) };
}
