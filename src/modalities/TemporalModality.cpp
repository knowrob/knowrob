//
// Created by danielb on 01.08.23.
//

#define EPISTEMIC_MODALITY_SINCE_KEY "since"
#define EPISTEMIC_MODALITY_UNTIL_KEY "until"

#include "knowrob/terms/Numeric.h"
#include "knowrob/modalities/TemporalModality.h"

using namespace knowrob;

TemporalModality::TemporalModality()
		: timeInterval_(std::nullopt), Modality() {}

TemporalModality::TemporalModality(const TimeInterval &timeInterval)
		: timeInterval_(timeInterval), Modality() {
	if (timeInterval_->since().has_value()) {
		parameters_[EPISTEMIC_MODALITY_SINCE_KEY] =
				std::make_shared<Double>(time::toSeconds(timeInterval_->since().value()));
	}
	if (timeInterval_->until().has_value()) {
		parameters_[EPISTEMIC_MODALITY_UNTIL_KEY] =
				std::make_shared<Double>(time::toSeconds(timeInterval_->until().value()));
	}
}

const std::optional<TimeInterval> &TemporalModality::timeInterval() const { return timeInterval_; }
