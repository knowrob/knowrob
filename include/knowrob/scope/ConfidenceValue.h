/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONFIDENCE_VALUE_H_
#define KNOWROB_CONFIDENCE_VALUE_H_

#include <ostream>

namespace knowrob {
	/**
	 * The amount of confidence for something represented within the range [0,1].
	 */
	class ConfidenceValue {
	public:
		/**
		 * @param value the confidence value in the range [0,1]
		 */
		explicit ConfidenceValue(double value);

		/**
		 * @return the maximum confidence value.
		 */
		static const ConfidenceValue& max();

		/**
		 * @return the minimum confidence value.
		 */
		static const ConfidenceValue& min();

		/**
		 * @return the confidence value in the range [0,1]
		 */
		const double& value() const { return value_; }

		/**
		 * @param other another confidence value.
		 * @return true if this confidence value is smaller than the other.
		 */
		bool operator<(const ConfidenceValue& other) const;

		/**
		 * @param other another confidence value.
		 * @return true if both values are the same
		 */
		bool operator==(const ConfidenceValue& other) const;

	protected:
		double value_;
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ConfidenceValue& confidence);
}

#endif //KNOWROB_CONFIDENCE_VALUE_H_
