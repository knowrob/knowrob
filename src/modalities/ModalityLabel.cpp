//
// Created by daniel on 17.07.23.
//

#include "knowrob/Logger.h"
#include "knowrob/modalities/ModalityLabel.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/modalities/EpistemicModality.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"

using namespace knowrob;

ModalityLabel::ModalityLabel(const ModalIteration &modalOperators)
: FormulaLabel()
{
    for(auto &x : modalOperators) {
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

ModalityLabel::ModalityLabel(const StatementData &tripleData)
: FormulaLabel()
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

std::shared_ptr<ModalityLabel> ModalityLabel::emptyLabel()
{
    static const auto empty = std::make_shared<ModalityLabel>(ModalIteration());
    return empty;
}

bool ModalityLabel::hasValue() const
{
	return epistemicOperator_.get() != nullptr || pastOperator_.get() != nullptr;
}

bool ModalityLabel::isAboutKnowledge() const
{
    return !isAboutBelief();
}

bool ModalityLabel::isAboutBelief() const
{
    return epistemicOperator_ ? epistemicOperator_->isModalPossibility() : false;
}

bool ModalityLabel::isAboutPresent() const
{
    return !isAboutPast();
}

bool ModalityLabel::isAboutSomePast() const
{
    return isAboutPast() && pastOperator_->isModalPossibility();
}

bool ModalityLabel::isAboutAllPast() const
{
    return isAboutPast() && pastOperator_->isModalNecessity();
}

bool ModalityLabel::isAboutPast() const
{
    return pastOperator_.get() != nullptr;
}

const std::optional<std::string>& ModalityLabel::agent() const
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

const std::optional<TimeInterval>& ModalityLabel::timeInterval() const
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

void ModalityLabel::setTimeInterval(const TimeInterval &ti)
{
	pastOperator_ = PastModality::P(ti);
}

bool ModalityLabel::operator==(const ModalityLabel &other) const
{
    if(this==&other) return true;

    if(!epistemicOperator_)            return !other.epistemicOperator_;
    else if(!other.epistemicOperator_) return false;
    else if(!(*epistemicOperator_ == *other.epistemicOperator_)) return false;

    if(!pastOperator_)            return !other.pastOperator_;
    else if(!other.pastOperator_) return false;
    else if(!(*pastOperator_ == *other.pastOperator_)) return false;

    return true;
}

bool ModalityLabel::isEqual(const FormulaLabel &other) const
{
    { return *this == *static_cast<const ModalityLabel*>(&other); } // NOLINT
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ModalityLabel& modality) //NOLINT
	{
		if(modality.epistemicOperator())
			os << *modality.epistemicOperator();
		if(modality.pastOperator())
			os << *modality.pastOperator();
		return os;
	}
}
