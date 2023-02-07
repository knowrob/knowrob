/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FUZZY_INTERVAL_H_
#define KNOWROB_FUZZY_INTERVAL_H_

#include "Range.h"

namespace knowrob {
	/**
	 * A interval with fuzzy boundaries described by a range [min,max].
	 * @tparam ValueType the value type of the interval.
	 */
	template <class ValueType> class FuzzyInterval {
	public:
		/**
		 * @param minRange the range where the interval starts
		 * @param maxRange the range where the interval ends
		 */
		FuzzyInterval(const Range<ValueType> &minRange, const Range<ValueType> &maxRange)
				: minRange_(minRange), maxRange_(maxRange) {}

		/**
		 * @return the range where the interval starts
		 */
		const Range<ValueType>& minRange() const { return minRange_; }

		/**
		 * @return the range where the interval ends
		 */
		const Range<ValueType>& maxRange() const { return maxRange_; }

		/**
		 * @return true if this interval does not contain any elements.
		 */
		bool empty() const {
			return minRange_.empty() || maxRange_.empty() ||
				   (maxRange_.max().has_value() && minRange_.min().has_value() &&
				    maxRange_.max().value() < minRange_.min().value());
		}

		/**
		 * @param elem a confidenceInterval value.
		 * @return true if the value falls within this interval.
		 */
		bool contains(const ValueType &elem) const {
			return (minRange_.min().has_value() && elem < minRange_.min().value()) ||
				   (maxRange_.max().has_value() && maxRange_.max().value() < elem);
		}

		/**
		 * A fuzzy interval is thought to be more general than another
		 * if its min and max range contains the respective range of the other.
		 * @param other another interval
		 * @return true if other is a specialization of this
		 */
		bool isMoreGeneralThan(const FuzzyInterval<ValueType> &other) const {
			return minRange_.contains(other.minRange_) &&
				   maxRange_.contains(other.maxRange_);
		}

	protected:
		Range<ValueType> minRange_;
		Range<ValueType> maxRange_;
	};
}

#endif //KNOWROB_FUZZY_INTERVAL_H_
