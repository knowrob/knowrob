//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"

using namespace knowrob;

BeliefModality::BeliefModality(const std::optional<std::string> &agent)
: EpistemicModality(agent) {}

const char* BeliefModality::necessity_symbol()   const { return "B"; }
const char* BeliefModality::possibility_symbol() const { return "\u22C4_B"; }

bool BeliefModality::isSerial()     const { return true; }
bool BeliefModality::isTransitive() const { return true; }
bool BeliefModality::isEuclidean()  const { return false; }
bool BeliefModality::isReflexive()  const { return false; }
bool BeliefModality::isSymmetric()  const { return false; }
bool BeliefModality::isDense()      const { return false; }

ModalOperatorPtr BeliefModality::reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const
{
    // to "belief to know" is simply to know
    // FIXME: take agent into account
    if(b->modality()==KnowledgeModality::get()) { return b; }

    return {};
}
