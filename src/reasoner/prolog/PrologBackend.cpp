/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/prolog/PrologBackend.h"
#include "knowrob/reasoner/prolog/PrologEngine.h"
#include "knowrob/reasoner/prolog/PrologTerm.h"

using namespace knowrob;

namespace knowrob {
	static const auto rdf_assert = "rdf_assert";
	static const auto rdf_retractall = "rdf_retractall";
	static const auto rdf_transaction = "rdf_transaction";
}

// TODO: Prolog allows the fourth parameter to be more than a string:
//    https://www.swi-prolog.org/pldoc/man?section=semweb-graphs
//    Potentially we could use this to store the whole KnowRob frame of a triple.

bool PrologBackend::initializeBackend(const PropertyTree &cfg) {
	return true;
}

bool PrologBackend::insertOne(const FramedTriple &triple) {
	// :- rdf_assert($triple.subject, $triple.predicate, $triple.object, $triple.origin).
	return PROLOG_ENGINE_EVAL(PrologTerm(triple, rdf_assert));
}

bool PrologBackend::removeOne(const FramedTriple &triple) {
	// :- rdf_retractall($triple.subject, $triple.predicate, $triple.object, $triple.origin).
	return PROLOG_ENGINE_EVAL(PrologTerm(triple, rdf_retractall));
}

bool PrologBackend::removeAllWithOrigin(std::string_view origin) {
	// :- rdf_retractall(_, _, _, $origin).
	return PROLOG_ENGINE_EVAL(PrologTerm(rdf_retractall, PrologTerm(), PrologTerm(), PrologTerm(), origin));
}

bool PrologBackend::insertAll(const TripleContainerPtr &triples) {
	// :- rdf_transaction(...).
	return PROLOG_ENGINE_EVAL(transaction(rdf_assert, triples));
}

bool PrologBackend::removeAll(const TripleContainerPtr &triples) {
	// :- rdf_transaction(...).
	return PROLOG_ENGINE_EVAL(transaction(rdf_retractall, triples));
}

PrologTerm PrologBackend::transaction(std::string_view rdf_functor, const TripleContainerPtr &triples) {
	// transactionTerm = rdf_transaction((rdf_functor(s, p, o, g), ...)).
	PrologTerm transactionGoal;
	for (const auto &triple: *triples) {
		transactionGoal = (transactionGoal & PrologTerm(*triple, rdf_functor));
	}
	return PrologTerm(rdf_transaction, transactionGoal);
}
