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
		negativeUngrounded_.emplace_back(predicate);
	} else {
		positiveUngrounded_.emplace_back(predicate);
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
	// TODO: should include predicates, but best ordering agnostic
	return val;
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
			os << '\t' << *x << '\n';
		}
		for(auto &x : negativeUngrounded_) {
			os << '\t' << "not "<< *x << '\n';
		}
	}
	return os;
}

std::string AnswerNo::toHumanReadableString() const {
	static const std::string longMsg = "there was evidence supporting the query to be false";
	return longMsg;
}

