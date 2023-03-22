/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_RANGE_H_
#define KNOWROB_RANGE_H_

#include <optional>

namespace knowrob {
	/**
	 * A value range [min,max]
	 * @tparam ValueType the type of values
	 */
	template <class ValueType> class Range {
	public:
		/**
		 * @param min the minimum value of the range
		 * @param max the maximum value of the range
		 */
		Range(const std::optional<ValueType> &min, const std::optional<ValueType> &max)
		: min_(min), max_(max) {}

		/**
		 * @return the minimum value of the range
		 */
		const std::optional<ValueType>& min() const { return min_; }

		/**
		 * @return the maximum value of the range
		 */
		const std::optional<ValueType>& max() const { return max_; }

		/**
		 * @return true if the range has no elements, i.e. max<min
		 */
		bool empty() const { return max_.has_value() && min_.has_value() && max_.value() < min_.value(); }

		/**
		 * @param other another range.
		 * @return true if this range isMoreGeneralThan another one.
		 */
		bool contains(const Range<ValueType> &other) const {
			return !((min_.has_value() && (!other.min_.has_value() || other.min_.value() < min_.value())) ||
					 (max_.has_value() && (!other.max_.has_value() || max_.value() < other.max_.value())));
		}

		/**
		 * Intersect this range with another one.
		 * @param other another range.
		 */
		Range<ValueType> intersectWith(const Range<ValueType> &other) const {
			auto *min = &min_;
			auto *max = &max_;
			if(other.min_.has_value() && (!min_.has_value() ||
				    min_.value() < other.min_.value())) {
				min = &other.min_;
			}
			if(other.max_.has_value() && (!max_.has_value() ||
					other.max_.value() < max_.value())) {
				max = &other.max_;
			}
			return { *min, *max };
		}

		bool hasValue() const { return min_.has_value() || max_.has_value(); }

	protected:
		std::optional<ValueType> min_;
		std::optional<ValueType> max_;
	};
}

#endif //KNOWROB_RANGE_H_
