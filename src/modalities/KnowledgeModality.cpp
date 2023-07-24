//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/BeliefModality.h"

using namespace knowrob;

KnowledgeModality::KnowledgeModality() : EpistemicModality() {}

KnowledgeModality::KnowledgeModality(const std::string_view &agent)
: EpistemicModality(agent) {}

bool KnowledgeModality::isSerial()     const { return true; }
bool KnowledgeModality::isReflexive()  const { return true; }
bool KnowledgeModality::isTransitive() const { return true; }
bool KnowledgeModality::isEuclidean()  const { return false; }
bool KnowledgeModality::isSymmetric()  const { return false; }
bool KnowledgeModality::isDense()      const { return false; }

ModalOperatorPtr KnowledgeModality::K()
{
    static auto modality = std::make_shared<KnowledgeModality>();
    static auto knowOperator =
        std::make_shared<ModalOperator>(modality, ModalOperatorType::NECESSITY);
    return knowOperator;
}

ModalOperatorPtr KnowledgeModality::K(const std::string_view &agent)
{
    auto modality = std::make_shared<KnowledgeModality>(agent);
    return std::make_shared<ModalOperator>(modality, ModalOperatorType::NECESSITY);
}

ModalOperatorPtr KnowledgeModality::reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const
{
    // "to know to belief" is simply "to belief"
    if(b->modality().modalityType()==ModalityType::Epistemic && b->isModalPossibility()) {
        auto *beliefModality = (BeliefModality*)&b->modality();
        if(this->agent() == beliefModality->agent()) return b;
    }

    return {};
}
