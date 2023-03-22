/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONFIDENCE_INTERVAL_H_
#define KNOWROB_CONFIDENCE_INTERVAL_H_

#include "Range.h"
#include "FuzzyInterval.h"
#include "ConfidenceValue.h"

namespace knowrob {
	/**
	 * A fuzzy confidence interval where min and end max confidence lie within a range.
	 */
	class ConfidenceInterval : public FuzzyInterval<ConfidenceValue> {
	public:
		/**
		 * @param minRange the value range for the minimum confidence
		 * @param maxRange the value range for the maximum confidence
		 */
		ConfidenceInterval(const Range<ConfidenceValue> &minRange, const Range<ConfidenceValue> &maxRange);

		/**
		 * @return an interval that only includes the maximum confidence value.
		 */
		static const ConfidenceInterval& certain();

		/**
		 * @return a confidence interval without any further constraints on min and max confidence values.
		 */
		static const ConfidenceInterval& any();

		/**
		 * @param value the minimum confidence value
		 * @return a confidence interval including all confidence values larger than the one provided
		 */
		static ConfidenceInterval at_least(const ConfidenceValue &value);
	};
}

#endif //KNOWROB_CONFIDENCE_INTERVAL_H_
