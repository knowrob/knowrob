/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/AnswerDontKnow.h"

namespace knowrob {
	const std::shared_ptr<const AnswerDontKnow> &GenericDontKnow() {
		static const auto instance = std::make_shared<const AnswerDontKnow>();
		return instance;
	}
} // namespace knowrob

using namespace knowrob;

AnswerDontKnow::AnswerDontKnow()
		: Answer() {
}

AnswerDontKnow::AnswerDontKnow(const AnswerDontKnow &other)
		: Answer(other) {
}

void AnswerDontKnow::setIsUncertain(std::optional<double> confidence) {
	confidence_ = confidence;
}

size_t AnswerDontKnow::hash() const {
	return Answer::hash();
}

std::ostream &AnswerDontKnow::write(std::ostream &os) const {
	return os << "don't know";
}

std::string AnswerDontKnow::toHumanReadableString() const {
	static const std::string longMsg = "there was no evidence to conclude either yes or no";
	return longMsg;
}
