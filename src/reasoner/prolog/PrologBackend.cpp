/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/BackendManager.h"
#include "knowrob/reasoner/prolog/PrologBackend.h"
#include "knowrob/reasoner/prolog/PrologEngine.h"
#include "knowrob/reasoner/prolog/PrologTerm.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

/**
 * Register the backend with the BackendManager
 */
KNOWROB_BUILTIN_BACKEND("Prolog:rdf_db", PrologBackend)

namespace knowrob {
	static const auto rdf_quad = "rdf";
	static const auto rdf_assert = "rdf_assert";
	static const auto rdf_retractall = "rdf_retractall";
	static const auto rdf_transaction = "rdf_transaction";
}

PrologBackend::PrologBackend()
		: QueryableBackend(BackendFeature::NothingSpecial) {
}

bool PrologBackend::initializeBackend() {
	PrologEngine::initializeProlog();
	return PROLOG_ENGINE_EVAL(
			PrologTerm("use_module",
			           PrologTerm("library", "semweb/rdf_db"),
			           PrologList({
			           		PrologTerm("/", "rdf", std::make_shared<Integer>(4)),
			           		PrologTerm("/", "rdf_assert", std::make_shared<Integer>(4)),
			           		PrologTerm("/", "rdf_retractall", std::make_shared<Integer>(4)),
			           		PrologTerm("/", "rdf_transaction", std::make_shared<Integer>(1)),
			           		PrologTerm("/", "rdf_has", std::make_shared<Integer>(3))
					   })));
}

bool PrologBackend::initializeBackend(const PropertyTree &cfg) {
	return initializeBackend();
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

bool PrologBackend::isPersistent() const {
	return false;
}

void PrologBackend::batch(const TripleHandler &callback) const {
	// :- rdf(?subject, ?property, ?object, ?origin).
	// Note: Prolog querying works via backtracking over term_t, and this
	// changes the value of the term_t in the Prolog engine such that imo we cannot
	// avoid copying the results into a container -- as opposed to mapping the memory
	// which was allocated by the Prolog engine during querying.
	// A copy of the term_t could be created though, but this seems pointless, so here
	// we just create new KnowRob terms with their own memory allocation.
	static auto var_s = std::make_shared<Variable>("s");
	static auto var_p = std::make_shared<Variable>("p");
	static auto var_o = std::make_shared<Variable>("o");
	static auto var_g = std::make_shared<Variable>("g");
	auto triples = std::make_shared<TripleViewBatch>(GlobalSettings::batchSize());

	PROLOG_ENGINE_QUERY(
			PrologTerm(rdf_quad,
					   PrologTerm(var_s),
					   PrologTerm(var_p),
					   PrologTerm(var_o),
					   PrologTerm(var_g)),
			[&](const BindingsPtr &bindings) {
				auto val_s = bindings->getAtomic(var_s->name());
				auto val_p = bindings->getAtomic(var_p->name());
				auto val_o = bindings->getAtomic(var_o->name());
				auto val_g = bindings->getAtomic(var_g->name());
				if(!val_s || !val_p || !val_o) {
					KB_WARN("Failed to retrieve triple from Prolog: missing grounding.");
					return;
				}

				FramedTriplePtr triple_ptr;
				triple_ptr.ptr = new FramedTripleCopy(val_s->stringForm(), val_p->stringForm());
				if (val_o->isNumeric() || val_o->isString()) {
					auto xsd_o = std::static_pointer_cast<XSDAtomic>(val_o);
					triple_ptr.ptr->setXSDValue(xsd_o->stringForm(), xsd_o->xsdType());
				} else if (val_o->isIRI()) {
					triple_ptr.ptr->setObjectIRI(val_o->stringForm());
				} else if (val_o->isBlank()) {
					triple_ptr.ptr->setObjectBlank(val_o->stringForm());
				}
				if (val_g) {
					triple_ptr.ptr->setGraph(val_g->stringForm());
				}
				triple_ptr.owned = true;

				triples->add(triple_ptr);
				if (triples->size() >= GlobalSettings::batchSize()) {
					callback(triples);
					triples->reset();
				}
			});

	// Clean up
	if (triples->size() > 0) {
		callback(triples);
	}
}

void PrologBackend::query(const GraphQueryPtr &query, const BindingsHandler &callback) {
	PROLOG_ENGINE_QUERY(PrologTerm(query), callback);
}

void PrologBackend::count(const ResourceCounter &callback) const {
	// :- sw_resource_frequency(-Resource, -Frequency)
	static const auto freq_f = "sw_resource_frequency";
	static auto var_res = std::make_shared<Variable>("resource");
	static auto var_freq = std::make_shared<Variable>("frequency");

	PROLOG_ENGINE_QUERY(PrologTerm(freq_f), [&callback](const BindingsPtr &bindings) {
		auto val_res = bindings->getAtomic(var_res->name());
		auto val_freq = bindings->getAtomic(var_freq->name());
		if (val_res && val_freq && val_freq->isNumeric()) {
			callback(val_res->stringForm(), std::static_pointer_cast<Numeric>(val_freq)->asInteger());
		}
	});
}
