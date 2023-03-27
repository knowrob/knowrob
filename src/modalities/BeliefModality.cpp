//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"

using namespace knowrob;

BeliefModality::BeliefModality()
: EpistemicModality() {}

const BeliefModality* BeliefModality::get()
{
    static BeliefModality instance;
    return &instance;
}

const char* BeliefModality::necessity_symbol()   const { return "B"; }
const char* BeliefModality::possibility_symbol() const { return "\u22C4_B"; }

const ModalOperatorPtr& BeliefModality::B()
{
    static const auto op = std::make_shared<ModalOperator>(
            get(), ModalOperatorType::NECESSITY);
    return op;
}

bool BeliefModality::isSerial()     const { return true; }
bool BeliefModality::isTransitive() const { return true; }
bool BeliefModality::isEuclidean()  const { return false; }
bool BeliefModality::isReflexive()  const { return false; }
bool BeliefModality::isSymmetric()  const { return false; }
bool BeliefModality::isDense()      const { return false; }

ModalOperatorPtr BeliefModality::reduce(const ModalOperatorPtr &a, const ModalOperatorPtr &b) const
{
    // to "belief to know" is simply to know
    if(b->modality()==KnowledgeModality::get()) { return b; }

    return {};
}
