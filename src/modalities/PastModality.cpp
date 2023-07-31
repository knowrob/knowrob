//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/PastModality.h"

using namespace knowrob;

PastModality::PastModality()
: TemporalModality()
{}

PastModality::PastModality(const TimeInterval &timeInterval)
: TemporalModality(timeInterval)
{}

ModalityType PastModality::modalityType() const { return ModalityType::Temporal_Past; }

const char* PastModality::necessity_symbol()   const { return "H"; }
const char* PastModality::possibility_symbol() const { return "P"; }

bool PastModality::isSerial()     const { return true; }
bool PastModality::isReflexive()  const { return true; }
bool PastModality::isTransitive() const { return true; }
bool PastModality::isDense()      const { return true; }
bool PastModality::isEuclidean()  const { return false; }
bool PastModality::isSymmetric()  const { return false; }

ModalOperatorPtr PastModality::P()
{
    static auto modality = std::make_shared<PastModality>();
    static auto beliefOperator =
        std::make_shared<ModalOperator>(modality, ModalOperatorType::POSSIBILITY);
    return beliefOperator;
}

ModalOperatorPtr PastModality::P(const TimeInterval &timeInterval)
{
    auto modality = std::make_shared<PastModality>(timeInterval);
    return std::make_shared<ModalOperator>(modality, ModalOperatorType::POSSIBILITY);
}

ModalOperatorPtr PastModality::H()
{
    static auto modality = std::make_shared<PastModality>();
    static auto beliefOperator =
        std::make_shared<ModalOperator>(modality, ModalOperatorType::NECESSITY);
    return beliefOperator;
}

ModalOperatorPtr PastModality::H(const TimeInterval &timeInterval)
{
    auto modality = std::make_shared<PastModality>(timeInterval);
    return std::make_shared<ModalOperator>(modality, ModalOperatorType::NECESSITY);
}
