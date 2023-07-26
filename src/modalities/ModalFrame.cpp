//
// Created by daniel on 17.07.23.
//

#include "knowrob/Logger.h"
#include "knowrob/modalities/ModalFrame.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/modalities/EpistemicModality.h"

using namespace knowrob;

ModalFrame::ModalFrame()
= default;

ModalFrame::ModalFrame(const ModalIteration &modalIteration)
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

bool ModalFrame::hasValue() const
{
	return epistemicOperator_.get() != nullptr || pastOperator_.get() != nullptr;
}

bool ModalFrame::isAboutKnowledge() const
{
    return !isAboutBelief();
}

bool ModalFrame::isAboutBelief() const
{
    return epistemicOperator_ ? epistemicOperator_->isModalPossibility() : false;
}

bool ModalFrame::isAboutPresent() const
{
    return !isAboutPast();
}

bool ModalFrame::isAboutSomePast() const
{
    return isAboutPast() && pastOperator_->isModalPossibility();
}

bool ModalFrame::isAboutAllPast() const
{
    return isAboutPast() && pastOperator_->isModalNecessity();
}

bool ModalFrame::isAboutPast() const
{
    return pastOperator_.get() != nullptr;
}

const std::optional<std::string>& ModalFrame::agent() const
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

const std::optional<TimeInterval>& ModalFrame::timeInterval() const
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

void ModalFrame::setTimeInterval(const TimeInterval &ti)
{
	pastOperator_ = PastModality::P(ti);
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ModalFrame& modality) //NOLINT
	{
		if(modality.epistemicOperator())
			os << *modality.epistemicOperator();
		if(modality.pastOperator())
			os << *modality.pastOperator();
		return os;
	}
}
