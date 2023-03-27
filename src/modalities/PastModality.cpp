//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/PastModality.h"

using namespace knowrob;

PastModality::PastModality()
: TemporalModality()
{}

const PastModality* PastModality::get()
{
    static PastModality instance;
    return &instance;
}

const ModalOperatorPtr& PastModality::P()
{
    static const auto op = std::make_shared<ModalOperator>(
            get(), ModalOperatorType::POSSIBILITY);
    return op;
}

const ModalOperatorPtr& PastModality::H() {
    static const auto op = std::make_shared<ModalOperator>(
            get(), ModalOperatorType::NECESSITY);
    return op;
}

const char* PastModality::necessity_symbol()   const { return "P"; }
const char* PastModality::possibility_symbol() const { return "H"; }

bool PastModality::isSerial()     const { return true; }
bool PastModality::isReflexive()  const { return true; }
bool PastModality::isTransitive() const { return true; }
bool PastModality::isDense()      const { return true; }
bool PastModality::isEuclidean()  const { return false; }
bool PastModality::isSymmetric()  const { return false; }
