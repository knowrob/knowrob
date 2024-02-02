/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_RDF_COMPUTABLE_H
#define KNOWROB_RDF_COMPUTABLE_H

#include "RDFLiteral.h"
#include "knowrob/reasoner/Reasoner.h"

namespace knowrob {
	class RDFComputable : public RDFLiteral {
	public:
		RDFComputable(const RDFLiteral &lit, const std::vector <std::shared_ptr<Reasoner>> &reasonerList)
				: RDFLiteral(lit), reasonerList_(reasonerList) {}

		const auto &reasonerList() const { return reasonerList_; }

	protected:
		std::vector <std::shared_ptr<Reasoner>> reasonerList_;
	};

	using RDFComputablePtr = std::shared_ptr<RDFComputable>;
}

#endif //KNOWROB_RDF_COMPUTABLE_H
