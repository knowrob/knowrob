//
// Created by daniel on 26.03.23.
//

#include "knowrob/modalities/PastModality.h"

using namespace knowrob;

PastModality::PastModality(const std::optional<TimeInterval> &timeInterval)
: TemporalModality(timeInterval)
{}

const char* PastModality::necessity_symbol()   const { return "P"; }
const char* PastModality::possibility_symbol() const { return "H"; }

bool PastModality::isSerial()     const { return true; }
bool PastModality::isReflexive()  const { return true; }
bool PastModality::isTransitive() const { return true; }
bool PastModality::isDense()      const { return true; }
bool PastModality::isEuclidean()  const { return false; }
bool PastModality::isSymmetric()  const { return false; }
