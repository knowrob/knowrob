/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/Answer.h>
#include "knowrob/Logger.h"
#include "knowrob/knowrob.h"
#include "knowrob/queries/AnswerYes.h"

using namespace knowrob;

size_t Answer::hash() const {
	size_t val = Token::hash();
	if (isNegative()) {
		hashCombine(val, 0);
	} else if (isPositive()) {
		hashCombine(val, 1);
	} else {
		hashCombine(val, 2);
	}
	return val;
}

namespace knowrob {
	AnswerPtr mergeAnswers(const AnswerPtr &a, const AnswerPtr &b, bool ignoreInconsistencies) {
		// a negative answer overrules any positive answer.
		// also for the moment we do not combine two negative answers
		if (a->isNegative()) {
			// a is "no"
			// TODO: merge negative answers
			return a;
		} else if (b->isNegative()) {
			// b is "no"
			// TODO: merge negative answers
			return b;
		} else if (a->isPositive()) {
			if (b->isPositive()) {
				// both positive -> combine
				auto a_positive = std::static_pointer_cast<const AnswerYes>(a);
				auto b_positive = std::static_pointer_cast<const AnswerYes>(b);
				return mergePositiveAnswers(a_positive, b_positive, ignoreInconsistencies);
			} else {
				// b is "don't know"
				return b;
			}
		} else {
			// b is "don't know"
			return a;
		}
	}

	bool AnswerComparator::operator()(const AnswerPtr &v0, const AnswerPtr &v1) const {
		if (!v0) {
			if (!v1) return false;
			else return true;
		} else if (!v1 || v0 == v1) {
			return false;
		} else if (v0->isCertain() != v1->isCertain()) {
			return v0->isCertain() < v1->isCertain();
		} else if (v0->isPositive() != v1->isPositive()) {
			return v0->isPositive() < v1->isPositive();
		} else if (v0->isNegative() != v1->isNegative()) {
			return v0->isNegative() > v1->isNegative();
		} else {
			return v0->hash() < v1->hash();
		}
	}
}
