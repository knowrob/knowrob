//
// Created by daniel on 17.07.23.
//

#include "knowrob/Logger.h"
#include "knowrob/modalities/ModalityFrame.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/modalities/EpistemicModality.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"

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

ModalityFrame::ModalityFrame(const StatementData &tripleData)
{
    // init epistemic operator
    auto epistemicOperator = tripleData.epistemicOperator;
    if(!epistemicOperator.has_value()) epistemicOperator = EpistemicOperator::KNOWLEDGE;
    if(epistemicOperator.value() == EpistemicOperator::BELIEF) {
        if(tripleData.agent) {
            if(tripleData.confidence.has_value()) {
                epistemicOperator_ = BeliefModality::B(tripleData.agent, tripleData.confidence.value());
            }
            else {
                epistemicOperator_ = BeliefModality::B(tripleData.agent);
            }
        }
        else if(tripleData.confidence.has_value()) {
            epistemicOperator_ = BeliefModality::B(tripleData.confidence.value());
        }
        else {
            epistemicOperator_ = BeliefModality::B();
        }
    }
    else { // EpistemicOperator::KNOWLEDGE
        if(tripleData.agent) epistemicOperator_ = KnowledgeModality::K(tripleData.agent);
        else                 epistemicOperator_ = KnowledgeModality::K();
    }

    // init time operator
    auto temporalOperator = tripleData.temporalOperator;
    if(!temporalOperator.has_value()) temporalOperator = TemporalOperator::ALWAYS;
    if(temporalOperator.value() == TemporalOperator::SOMETIMES) {
        if(tripleData.begin.has_value() || tripleData.end.has_value()) {
            pastOperator_ = PastModality::P(TimeInterval(tripleData.begin, tripleData.end));
        }
        else {
            pastOperator_ = PastModality::P();
        }
    }
    else {
        if(tripleData.begin.has_value() || tripleData.end.has_value()) {
            pastOperator_ = PastModality::H(TimeInterval(tripleData.begin, tripleData.end));
        }
        else {
            pastOperator_ = PastModality::H();
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
