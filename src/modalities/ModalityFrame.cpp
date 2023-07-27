//
// Created by daniel on 17.07.23.
//

#include "knowrob/Logger.h"
#include "knowrob/modalities/ModalityFrame.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/modalities/EpistemicModality.h"

using namespace knowrob;

ModalityFrame::ModalityFrame()
= default;

ModalityFrame::ModalityFrame(const ModalIteration &modalIteration)
{
    for(auto &x : modalIteration) {
        if(x->modality().modalityType() == ModalityType::Epistemic) {
            epistemicOperator_ = x;
        }
        else if(x->modality().modalityType() == ModalityType::Temporal_Past) {
            pastOperator_ = x;
        }
        else {
            KB_WARN("unexpected modalFrame");
        }
    }
}

bool ModalityFrame::hasValue() const
{
	return epistemicOperator_.get() != nullptr || pastOperator_.get() != nullptr;
}

bool ModalityFrame::isAboutKnowledge() const
{
    return !isAboutBelief();
}

bool ModalityFrame::isAboutBelief() const
{
    return epistemicOperator_ ? epistemicOperator_->isModalPossibility() : false;
}

bool ModalityFrame::isAboutPresent() const
{
    return !isAboutPast();
}

bool ModalityFrame::isAboutSomePast() const
{
    return isAboutPast() && pastOperator_->isModalPossibility();
}

bool ModalityFrame::isAboutAllPast() const
{
    return isAboutPast() && pastOperator_->isModalNecessity();
}

bool ModalityFrame::isAboutPast() const
{
    return pastOperator_.get() != nullptr;
}

const std::optional<std::string>& ModalityFrame::agent() const
{
    if(epistemicOperator_) {
        auto epistemicModality = (EpistemicModality*)&epistemicOperator_->modality();
        return epistemicModality->agent();
    }
    else {
        static std::optional<std::string> empty;
        return empty;
    }
}

const std::optional<TimeInterval>& ModalityFrame::timeInterval() const
{
    if(pastOperator_) {
        auto pastModality = (PastModality*)&pastOperator_->modality();
        return pastModality->timeInterval();
    }
    else {
        static std::optional<TimeInterval> empty;
        return empty;
    }
}

void ModalityFrame::setTimeInterval(const TimeInterval &ti)
{
	pastOperator_ = PastModality::P(ti);
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ModalityFrame& modality) //NOLINT
	{
		if(modality.epistemicOperator())
			os << *modality.epistemicOperator();
		if(modality.pastOperator())
			os << *modality.pastOperator();
		return os;
	}
}
