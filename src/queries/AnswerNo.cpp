/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/AnswerNo.h"
#include "knowrob/knowrob.h"

namespace knowrob {
	const std::shared_ptr<const AnswerNo> &GenericNo() {
		static const auto instance = std::make_shared<const AnswerNo>();
		return instance;
	}
} // namespace knowrob

using namespace knowrob;

AnswerNo::AnswerNo()
		: Answer(),
		  isUncertain_(false) {
}

AnswerNo::AnswerNo(const AnswerNo &other)
		: Answer(other),
		  isUncertain_(other.isUncertain_) {
}

void AnswerNo::addUngrounded(const std::shared_ptr<Predicate> &predicate, bool isNegated) {
	if (isNegated) {
		negativeUngrounded_.emplace_back(predicate, DefaultGraphSelector(), reasonerTerm_);
	} else {
		positiveUngrounded_.emplace_back(predicate, DefaultGraphSelector(), reasonerTerm_);
	}
}

bool AnswerNo::isUncertain() const {
	return isUncertain_;
}

void AnswerNo::setIsUncertain(std::optional<double> confidence) {
	isUncertain_ = true;
	if (confidence.has_value()) {
		confidence_ = confidence;
	}
}

size_t AnswerNo::hash() const {
	size_t val = Answer::hash();
	hashCombine(val, isUncertain_);
	hashCombine(val, std::hash<std::optional<double>>{}(confidence_));
	// FIXME: must include predicates in hash. best to keep them in a sorted container.
	return val;
}

bool AnswerNo::mergeWith(const AnswerNo &other) {
	reasonerTerm_ = {};
	isUncertain_ = isUncertain_ || other.isUncertain_;
	// insert groundings of other answer
	positiveUngrounded_.insert(positiveUngrounded_.end(),
							   other.positiveUngrounded_.begin(), other.positiveUngrounded_.end());
	negativeUngrounded_.insert(negativeUngrounded_.end(),
							   other.negativeUngrounded_.begin(), other.negativeUngrounded_.end());
	return true;
}

std::ostream &AnswerNo::write(std::ostream &os) const {
	if(reasonerTerm_) {
		os << "[" << reasonerTerm_->value() << "] ";
	}
	if(isUncertain()) {
		os << "probably ";
	}
	os << "no";
	if(!positiveUngrounded_.empty() || !negativeUngrounded_.empty()) {
		os << ", because:\n";
		for(auto &x : positiveUngrounded_) {
			os << '\t' << *x.graphSelector() << ' ' << '~' << *x.predicate();
			if(x.reasonerTerm() && x.reasonerTerm() != reasonerTerm_) {
				os << ' ' << '[' << x.reasonerTerm()->value() << "]";
			}
			os << '\n';
		}
		for(auto &x : negativeUngrounded_) {
			os << '\t' << *x.graphSelector() << *x.predicate();
			if(x.reasonerTerm() && x.reasonerTerm() != reasonerTerm_) {
				os << ' ' << '[' << x.reasonerTerm()->value() << "]";
			}
			os << '\n';
		}
	} else {
		os << '\n';
	}
	return os;
}

std::string AnswerNo::toHumanReadableString() const {
	static const std::string longMsg = "there was evidence supporting the query to be false";
	return longMsg;
}

namespace knowrob {
	AnswerPtr mergeNegativeAnswers(const AnswerNoPtr &a, const AnswerNoPtr &b) {
		auto mergedAnswer = std::make_shared<AnswerNo>(*a);
		if (mergedAnswer->mergeWith(*b)) {
			return mergedAnswer;
		} else {
			// merging failed
			return {};
		}
	}
} // knowrob
