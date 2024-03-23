/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_RDF_COMPUTABLE_H
#define KNOWROB_RDF_COMPUTABLE_H

#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/reasoner/GoalDrivenReasoner.h"

namespace knowrob {
	class RDFComputable : public FramedTriplePattern {
	public:
		RDFComputable(const FramedTriplePattern &lit, const std::vector <std::shared_ptr<GoalDrivenReasoner>> &reasonerList)
				: FramedTriplePattern(lit), reasonerList_(reasonerList) {}

		const auto &reasonerList() const { return reasonerList_; }

	protected:
		std::vector <std::shared_ptr<GoalDrivenReasoner>> reasonerList_;
	};

	using RDFComputablePtr = std::shared_ptr<RDFComputable>;
}

#endif //KNOWROB_RDF_COMPUTABLE_H
