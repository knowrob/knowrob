//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"

using namespace knowrob;

BeliefModality::BeliefModality() : EpistemicModality() {}

BeliefModality::BeliefModality(const std::string_view &agent)
: EpistemicModality(agent) {}

bool BeliefModality::isSerial()     const { return true; }
bool BeliefModality::isTransitive() const { return true; }
bool BeliefModality::isEuclidean()  const { return false; }
bool BeliefModality::isReflexive()  const { return false; }
bool BeliefModality::isSymmetric()  const { return false; }
bool BeliefModality::isDense()      const { return false; }

ModalOperatorPtr BeliefModality::B()
{
    static auto modality = std::make_shared<BeliefModality>();
    static auto beliefOperator =
        std::make_shared<ModalOperator>(modality, ModalOperatorType::POSSIBILITY);
    return beliefOperator;
}

ModalOperatorPtr BeliefModality::B(const std::string_view &agent)
{
    auto modality = std::make_shared<BeliefModality>(agent);
    return std::make_shared<ModalOperator>(modality, ModalOperatorType::POSSIBILITY);
}

ModalOperatorPtr BeliefModality::reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const
{
    // to "belief to know" is simply to know
    if(b->modality().modalityType()==ModalityType::Epistemic && b->isModalNecessity()) {
        auto *knowledgeModality = (KnowledgeModality*)&b->modality();
        if(this->agent() == knowledgeModality->agent()) return b;
    }

    return {};
}
