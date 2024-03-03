/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REIFICATION_MODEL_H
#define KNOWROB_REIFICATION_MODEL_H

#include "string"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob::reification {
	const AtomPtr individualPrefix = Atom::Tabled("http://knowrob.org/kb/reified.owl#");
	const IRIAtomPtr ReifiedRelation = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#ReifiedRelation");
	const IRIAtomPtr hasSubject = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasSubject");
	const IRIAtomPtr hasObject = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasObject");
	const IRIAtomPtr hasLiteral = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasLiteral");
	const IRIAtomPtr isUncertain = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#isUncertain");
	const IRIAtomPtr isOccasional = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#isOccasional");
	const IRIAtomPtr hasPerspective = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasPerspective");
	const IRIAtomPtr hasConfidence = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasConfidence");
	const IRIAtomPtr hasBeginTime = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasBeginTime");
	const IRIAtomPtr hasEndTime = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasEndTime");
}
// knowrob

#endif //KNOWROB_REIFICATION_MODEL_H
