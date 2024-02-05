/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/reasoner/Reasoner.h"

using namespace knowrob;

Reasoner::Reasoner()
: reasonerManagerID_(0)
{
}

void Reasoner::setReasonerManager(uint32_t managerID)
{
    reasonerManagerID_ = managerID;
}

void Reasoner::setInferredTriples(const std::vector<StatementData> &inferredSet)
{
	KB_WARN("TODO: add inferred {}", inferredSet.size());
	// TODO: find all triples that newly appear in the set of inferred triples,
	//       store it in a vector and call insert on the KB.
	//       but try to not call the reasoner that generated the triples.
	// TODO: find all triples that are no longer in the set of inferred triples,
	//       store it in a vector and call remove on the KB.
	//       but try to not call the reasoner that generated the triples.
}

PredicateDescriptionPtr Reasoner::getLiteralDescription(const RDFLiteral &literal)
{
	if(literal.propertyTerm()->type() == TermType::STRING) {
		auto p = std::static_pointer_cast<StringTerm>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->value(), 2));
	}
	else {
		return {};
	}
}
