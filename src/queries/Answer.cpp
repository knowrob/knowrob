/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/Answer.h>
#include "knowrob/Logger.h"
#include "knowrob/knowrob.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/AnswerNo.h"

using namespace knowrob;

void Answer::applyFrame(const GraphSelector &frame) {
	frame_->graph = frame.graph;
	frame_->perspective = frame.perspective;
	frame_->confidence = std::nullopt;
	frame_->begin = frame.begin;
	frame_->end = frame.end;
	frame_->uncertain = frame.uncertain;
	frame_->occasional = frame.occasional;
}

bool Answer::isUncertain() const {
	if (frame_->confidence.has_value()) {
		if (frame_->confidence.value() < 1.0) return true;
	}
	if (frame_->uncertain) {
		return true;
	}
	return false;
}

bool Answer::isOccasionallyTrue() const {
	return frame_->occasional;
}

void Answer::setIsUncertain(bool val, std::optional<double> confidence) {
	frame_->uncertain = val;
	if (val) {
		if (confidence.has_value()) {
			frame_->confidence = confidence;
		}
	} else {
		frame_->confidence = 1.0;
	}
}

void Answer::setIsOccasionallyTrue(bool val) {
	frame_->occasional = val;
}

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
		// also for the moment we do not combine two negative answers
		if (a->isNegative()) {
			if (b->isNegative()) {
				// both are on
				auto a_negative = std::static_pointer_cast<const AnswerNo>(a);
				auto b_negative = std::static_pointer_cast<const AnswerNo>(b);
				return mergeNegativeAnswers(a_negative, b_negative);
			} else {
				// a is "no"
				return a;
			}
		} else if (b->isNegative()) {
			// b is "no"
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
