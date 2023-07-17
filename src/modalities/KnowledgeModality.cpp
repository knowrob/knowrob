//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/BeliefModality.h"

using namespace knowrob;

KnowledgeModality::KnowledgeModality(const std::optional<std::string> &agent)
: EpistemicModality(agent) {}

const KnowledgeModality* KnowledgeModality::get()
{
    static KnowledgeModality instance;
    return &instance;
}

const ModalOperatorPtr& KnowledgeModality::K()
{
    static const auto op = std::make_shared<ModalOperator>(
            get(), ModalOperatorType::NECESSITY);
    return op;
}

const char* KnowledgeModality::necessity_symbol()   const { return "K"; }
const char* KnowledgeModality::possibility_symbol() const { return "\u22C4_K"; }

bool KnowledgeModality::isSerial()     const { return true; }
bool KnowledgeModality::isReflexive()  const { return true; }
bool KnowledgeModality::isTransitive() const { return true; }
bool KnowledgeModality::isEuclidean()  const { return false; }
bool KnowledgeModality::isSymmetric()  const { return false; }
bool KnowledgeModality::isDense()      const { return false; }

ModalOperatorPtr KnowledgeModality::reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const
{
    // "to know to belief" is simply "to belief"
    // FIXME: take agent into account
    if(b->modality()==BeliefModality::get()) { return b; }

    return {};
}
