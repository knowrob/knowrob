/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/db/QueryableBackend.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/UnReificationContainer.h"
#include "knowrob/reification/ReifiedQuery.h"

using namespace knowrob;

AtomPtr QueryableBackend::versionProperty = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasVersionOfOrigin");

QueryableBackend::QueryableBackend() : batchSize_(1000) {
}

std::vector<std::string> QueryableBackend::getOrigins() {
	static auto v_origin = std::make_shared<Variable>("Origin");
	static auto v_version = std::make_shared<Variable>("Version");
	std::vector<std::string> origins;
	matchDirect(FramedTriplePattern(v_origin, versionProperty, v_version),
				[&](const FramedTriple &triple) { origins.emplace_back(triple.subject()); });
	return origins;
}

void QueryableBackend::setVersionOfOrigin(std::string_view origin, std::string_view version) {
	FramedTripleView triple;
	triple.setSubject(origin);
	triple.setPredicate(versionProperty->stringForm());
	triple.setStringValue(version);
	triple.setGraph(origin);
	insertOne(triple);
}

std::optional<std::string> QueryableBackend::getVersionOfOrigin(std::string_view origin) {
	static auto v_version = std::make_shared<Variable>("Version");
	std::optional<std::string> version;
	matchDirect(FramedTriplePattern(std::make_shared<Atom>(origin), versionProperty, v_version),
				[&](const FramedTriple &triple) { version = triple.createStringValue(); });
	return version;
}

void QueryableBackend::foreach(const semweb::TripleVisitor &visitor) const {
	if (canStoreTripleContext()) {
		foreachDirect(visitor);
		return;
	}
	// fill a container that reverses a reification.
	UnReificationContainer unReifiedTriples;
	// the UnReificationContainer uses triple views, but memory of original triples can be
	// lost in the loop. So we need to store the original triples in a vector, and create
	// a view on them in the UnReificationContainer.
	// TODO: copy of original triples can be avoided by switching to FramedTriplePtr and taking over ownership in the loop.
	std::vector<FramedTripleCopy> originalTriples;
	// finally loop over all original triples
	foreachDirect([&](const FramedTriple &triple) {
		if (ReifiedTriple::isPartOfReification(triple)) {
			auto &copy = originalTriples.emplace_back(triple);
			unReifiedTriples.add(copy);
		} else {
			visitor(triple);
		}
	});
	// after looping over all original triples, also visit the un-reified ones
	for (auto &triple: unReifiedTriples) {
		visitor(*triple.ptr);
	}
}

void QueryableBackend::batch(const semweb::TripleHandler &callback) const {
	if (canStoreTripleContext()) {
		batchDirect(callback);
		return;
	}
	// fill a container that reverses a reification.
	UnReificationContainer unReifiedTriples;
	// take over ownership of triples in batches that need to be reified.
	// note: reified triples could be split into multiple batches which makes
	// the collapsing of them more difficult.
	// to this end we defer the collapsing until the batchDirect call has completed
	// while taking over ownership of the reified triples to avoid copies and allow
	// the use of views in the UnReificationContainer.
	std::vector<FramedTriplePtr> reificationTriples;
	auto batch = std::make_shared<semweb::TripleViewBatch>(batchSize_);
	batchDirect([&](const semweb::TripleContainerPtr &triples) {
		for (auto &triple: *triples) {
			if (ReifiedTriple::isPartOfReification(*triple.ptr)) {
				// take over ownership of triple
				if (triple.owned) {
					triple.owned = false;
					auto &newOwner = reificationTriples.emplace_back(triple.ptr);
					unReifiedTriples.add(*newOwner.ptr);
				} else {
					auto &copy = reificationTriples.emplace_back(new FramedTripleCopy(*triple.ptr));
					unReifiedTriples.add(*copy.ptr);
				}
			} else {
				batch->add(triple);
				if (batch->size() >= batchSize_) {
					callback(batch);
					batch->reset();
				}
			}
		}
	});
	for (auto &triple: unReifiedTriples) {
		batch->add(triple);
		if (batch->size() >= batchSize_) {
			callback(batch);
			batch->reset();
		}
	}
	if (!batch->empty()) {
		callback(batch);
	}
}

bool QueryableBackend::contains(const FramedTriple &triple) {
	if (!canStoreTripleContext()) {
		ReifiedTriple reification(triple, vocabulary());
		bool containsAll = false;
		for (auto &reified: reification) {
			containsAll = containsDirect(*reified.ptr);
			if (!containsAll) {
				break;
			}
		}
		return containsAll;
	} else {
		return containsDirect(triple);
	}
}

void QueryableBackend::match(const FramedTriplePattern &q, const semweb::TripleVisitor &visitor) {
	static auto ctx = std::make_shared<QueryContext>();
	if (!canStoreTripleContext() && ReifiedQuery::isReifiable(q)) {
		// FIXME: must expand into conjunction of atomic disjunctions of two literals
		auto reified = std::make_shared<ReifiedQuery>(q, vocabulary());
		queryDirect(reified, [&](const FramedBindingsPtr &bindings) {
			FramedTripleView triple;
			if (q.instantiateInto(triple, bindings)) {
				visitor(triple);
			}
		});
	} else {
		matchDirect(q, visitor);
	}
}

void QueryableBackend::query(const ConjunctiveQueryPtr &q, const FramedBindingsHandler &callback) {
	// TODO: why does the query have a frame, plus each of its literals?
	//       do really need both be taken into account here? I guess the meaning is that each literal
	//       has the same query frame? but query backends might ust iterate over the literals.
	if (!canStoreTripleContext() && ReifiedQuery::isReifiable(q)) {
		// FIXME: must expand into conjunction of atomic disjunctions of two literals
		// TODO: maybe also need to do some special expansion magic for adding context variables to construct the
		//       answer frame from bindings.
		auto reified = std::make_shared<ReifiedQuery>(q, vocabulary());
		queryDirect(reified, [&](const FramedBindingsPtr &bindings) {
			// TODO: computation of frame would need to be implemented here.
			//       for this we can iterate over literals and apply bindings to them to consolidate
			//       an overall frame.
			//       but would also be good if the process can be constrained e.g. like in mongolog
			//       where solutions are skipped if time intervals are not overlapping.
			//       but this is difficult, e.g. in sparql this might be possible with FILTER.
			//       but well post processing of the results is also possible more easily.
			//       is this really feasible? e.g. when only temporal operator is used then no variables would
			//       exist to be bound to the time interval. but well they could be added in the reification step.
			//       but what if we have a temporal operator plus a fixed time interval? well we could just
			//       return the fixed interval as solution without specializing it, that would be simple.
			//       but we could also use a mechanism like in mongolog `<(4)->Var` where terms can have a value
			//       and a variable part in this case. That way sparql could be able to handle this.
			callback(bindings);
		});
	} else {
		queryDirect(q, callback);
	}
}

void QueryableBackend::evaluateQuery(const ConjunctiveQueryPtr &q, const TokenBufferPtr &resultStream) {
	auto channel = TokenStream::Channel::create(resultStream);
	try {
		bool hasPositiveAnswer = false;
		query(q, [&](const FramedBindingsPtr &bindings) {
			channel->push(yes(q, bindings));
			hasPositiveAnswer = true;
		});
		if (!hasPositiveAnswer) {
			channel->push(no(q));
		}
		channel->push(EndOfEvaluation::get());
	}
	catch (const std::exception &e) {
		// make sure EOS is pushed to the stream
		channel->push(EndOfEvaluation::get());
		throw;
	}
}

TokenBufferPtr QueryableBackend::submitQuery(const ConjunctiveQueryPtr &q) {
	std::shared_ptr<TokenBuffer> result = std::make_shared<TokenBuffer>();
	auto runner =
			std::make_shared<ThreadPool::LambdaRunner>(
					[this, q, result](const ThreadPool::LambdaRunner::StopChecker &) {
						evaluateQuery(q, result);
					});
	DefaultThreadPool()->pushWork(runner, [result, q](const std::exception &e) {
		KB_WARN("an exception occurred for graph query ({}): {}.", *q, e.what());
		result->close();
	});
	return result;
}

AnswerPtr QueryableBackend::yes(const ConjunctiveQueryPtr &q, const FramedBindingsPtr &bindings) {
	static const auto edbTerm = Atom::Tabled("EDB");
	auto positiveAnswer = std::make_shared<AnswerYes>(bindings);
	positiveAnswer->setReasonerTerm(edbTerm);
	if (bindings->frame()) {
		positiveAnswer->setFrame(bindings->frame());
	}
	// add predicate groundings to the answer
	for (auto &rdfLiteral: q->literals()) {
		auto p = rdfLiteral->predicate();
		auto p_instance = applyBindings(p, *positiveAnswer->substitution());
		positiveAnswer->addGrounding(
				std::static_pointer_cast<Predicate>(p_instance),
				bindings->frame(),
				rdfLiteral->isNegated());
	}
	return positiveAnswer;
}

AnswerPtr QueryableBackend::no(const ConjunctiveQueryPtr &q) {
	static const auto edbTerm = Atom::Tabled("EDB");
	// send one negative answer if no positive answer was found
	auto negativeAnswer = std::make_shared<AnswerNo>();
	negativeAnswer->setReasonerTerm(edbTerm);
	// the answer is uncertain as we only were not able to obtain a positive answer
	// which does not mean that there is no positive answer.
	negativeAnswer->setIsUncertain(true);
	// add ungrounded literals to negative answer.
	// but at the moment the information is lost at which literal the query failed.
	// TODO: would be great if we could report the failing literal.
	//       but seems hard to provide this information in the current framework.
	//       it could be queried here, but that seems a bit costly.
	// well at least we know if it is a single literal.
	if (q->literals().size() == 1) {
		negativeAnswer->addUngrounded(q->literals().front()->predicate(),
									  q->literals().front()->isNegated());
	}
	return negativeAnswer;
}
