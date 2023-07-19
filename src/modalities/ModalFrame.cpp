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
            KB_WARN("unexpected modality");
        }
    }
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
        return {};
    }
}
