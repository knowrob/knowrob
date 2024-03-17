/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/BackendInterface.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/UnReificationContainer.h"
#include "knowrob/reification/ReifiedQuery.h"
#include "knowrob/db/BackendTransaction.h"
#include "knowrob/triples/GraphBuiltin.h"
#include "knowrob/knowrob.h"
#include "knowrob/reification/model.h"

using namespace knowrob;

std::shared_ptr<transaction::Transaction> BackendInterface::createTransaction(
		const QueryableBackendPtr &queryable,
		TransactionType transactionType,
		BackendSelection transactionTargets,
		const std::vector<std::shared_ptr<DefinedBackend>> &backends) {
	std::shared_ptr<transaction::Transaction> transaction;
	if (transactionType == Insert) {
		transaction = std::make_shared<transaction::Insert>(queryable, vocabulary());
	} else {
		transaction = std::make_shared<transaction::Remove>(queryable, vocabulary());
	}
	if (transactionTargets == Including) {
		for (auto &backend: backends) {
			transaction->addBackend(backend);
		}
	} else {
		for (auto &definedBackend: backendManager_->backendPool()) {
			auto &backend = definedBackend.second->backend();

			bool skip = false;
			if (transactionTargets == Excluding) {
				for (auto &excluded: backends) {
					if (excluded && backend == excluded->backend()) {
						skip = true;
						break;
					}
				}
			}
			if (skip) continue;
			transaction->addBackend(definedBackend.second);
		}
	}
	return transaction;
}

bool BackendInterface::removeAllWithOrigin(std::string_view origin) {
	// remove all triples with a given origin from all backends.
	std::vector<std::shared_ptr<ThreadPool::Runner>> transactions;
	for (auto &it: backendManager_->backendPool()) {
		auto definedBackend = it.second;
		// create a worker goal that performs the transaction
		auto transaction = std::make_shared<ThreadPool::LambdaRunner>(
				[definedBackend, origin](const ThreadPool::LambdaRunner::StopChecker &) {
					if (definedBackend->backend()->removeAllWithOrigin(origin)) {
						// unset version of origin in backend
						definedBackend->setVersionOfOrigin(origin, std::nullopt);
					} else {
						KB_WARN("removal of triples with origin '{}' from backend '{}' failed!", origin,
								definedBackend->name());
					}
				});
		// push goal to thread pool
		DefaultThreadPool()->pushWork(
				transaction,
				[definedBackend](const std::exception &exc) {
					KB_ERROR("transaction failed for backend '{}': {}", definedBackend->name(), exc.what());
				});
		transactions.push_back(transaction);
	}

	// wait for all transactions to finish
	for (auto &transaction: transactions) transaction->join();

	// update vocabulary: select all terms that are defined in a given origin,
	// and then remove them from the vocabulary.
	// TODO: only do this if no other backend defines the same origin?
	// remove origin from import hierarchy
	if (!vocabulary()->importHierarchy()->isReservedOrigin(origin)) {
		vocabulary()->importHierarchy()->removeCurrentGraph(origin);
	}

	return true;
}

bool BackendInterface::mergeInsert(const QueryableBackendPtr &backend, const FramedTriple &triple) {
	auto pat = std::make_shared<FramedTriplePattern>(triple);
	// Match triples where interval intersection is not empty
	pat->setIsOccasionalTerm(groundable(Numeric::trueAtom()));
	// Construct a merged triple
	FramedTripleView mergedTriple(triple);
	// TODO: copy of original triples can be avoided by switching to FramedTriplePtr and taking over ownership in the loop.
	// TODO: query for individual name of reified triple and hand it to remove.
	//       can be done by using query of original backend, or if the query can be changed to return the id
	std::vector<FramedTriplePtr> overlappingTriples;
	match(backend, *pat, [&](const FramedTriple &matchedTriple) {
		if (mergedTriple.mergeFrame(matchedTriple)) {
			auto &x = overlappingTriples.emplace_back();
			x.ptr = new FramedTripleCopy(matchedTriple);
			x.owned = true;
		}
	});
	if (!overlappingTriples.empty()) {
		// remove overlapping triples if any
		auto container = std::make_shared<ProxyTripleContainer>(&overlappingTriples);
		createTransaction(backend, Remove)->commit(container);
	}
	// Insert the triple after merging with overlapping existing ones
	createTransaction(backend, Insert)->commit(mergedTriple);
	return true;
}

bool BackendInterface::contains(const QueryableBackendPtr &backend, const FramedTriple &triple) const {
	if (backend->supports(BackendFeature::TripleContext)) {
		return backend->contains(triple);
	}

	ReifiedTriple reification(triple, vocabulary());
	bool containsAll = false;
	for (auto &reified: reification) {
		containsAll = backend->contains(*reified.ptr);
		if (!containsAll) {
			break;
		}
	}
	return containsAll;
}

void BackendInterface::foreach(const QueryableBackendPtr &backend, const TripleVisitor &visitor) {
	if (backend->supports(BackendFeature::TripleContext)) {
		backend->foreach(visitor);
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
	backend->foreach([&](const FramedTriple &triple) {
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

void BackendInterface::batch(const QueryableBackendPtr &backend, const TripleHandler &callback) {
	if (backend->supports(BackendFeature::TripleContext)) {
		backend->batch(callback);
		return;
	}

	auto batchSize = GlobalSettings::batchSize();
	// fill a container that reverses a reification.
	UnReificationContainer unReifiedTriples;
	// take over ownership of triples in batches that need to be reified.
	// note: reified triples could be split into multiple batches which makes
	// the collapsing of them more difficult.
	// to this end we defer the collapsing until the batchDirect call has completed
	// while taking over ownership of the reified triples to avoid copies and allow
	// the use of views in the UnReificationContainer.
	std::vector<FramedTriplePtr> reificationTriples;
	auto batch = std::make_shared<TripleViewBatch>(batchSize);
	backend->batch([&](const TripleContainerPtr &triples) {
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
				if (batch->size() >= batchSize) {
					callback(batch);
					batch->reset();
				}
			}
		}
	});
	for (auto &triple: unReifiedTriples) {
		batch->add(triple);
		if (batch->size() >= batchSize) {
			callback(batch);
			batch->reset();
		}
	}
	if (!batch->empty()) {
		callback(batch);
	}
}

static void setReifiedVariables( // NOLINT(misc-no-recursion)
		const std::shared_ptr<GraphTerm> &t,
		const std::map<std::string_view, VariablePtr> &variables) {
	switch (t->termType()) {
		case knowrob::GraphTermType::Pattern: {
			auto &pattern = std::static_pointer_cast<GraphPattern>(t)->value();
			if (!pattern->propertyTerm() || !pattern->propertyTerm()->isAtomic()) break;
			auto atomic = std::static_pointer_cast<Atomic>(pattern->propertyTerm());
			auto needle = variables.find(atomic->stringForm());
			if (needle != variables.end()) {
				pattern->setObjectVariable(needle->second);
			}
			break;
		}
		case knowrob::GraphTermType::Union:
		case knowrob::GraphTermType::Sequence: {
			auto connective = std::static_pointer_cast<GraphConnective>(t);
			for (auto &term: connective->terms()) {
				setReifiedVariables(term, variables);
			}
			break;
		}
		case knowrob::GraphTermType::Builtin:
			break;
	};
}

void BackendInterface::match(const QueryableBackendPtr &backend, const FramedTriplePattern &q,
							 const TripleVisitor &visitor) const {
	static auto ctx = std::make_shared<QueryContext>();
	if (backend->supports(BackendFeature::TripleContext)) {
		backend->match(q, visitor);
	} else {
		auto flags = ReifiedQuery::getReificationFlags(q);
		if (flags & IncludeOriginal) {
			backend->match(q, visitor);
		}
		if (flags & IncludeReified) {
			static auto v_o = std::make_shared<Variable>("o");
			static auto v_begin = std::make_shared<Variable>("begin");
			static auto v_end = std::make_shared<Variable>("end");
			static auto v_confidence = std::make_shared<Variable>("confidence");
			static auto v_uncertain = std::make_shared<Variable>("uncertain");
			static auto v_occasional = std::make_shared<Variable>("occasional");

			auto reified = std::make_shared<ReifiedQuery>(q, vocabulary(), true);
			// insert variables for contextual parameters
			setReifiedVariables(reified->term(), {
					{reification::hasBeginTime->stringForm(),  v_begin},
					{reification::hasEndTime->stringForm(),    v_end},
					{reification::hasConfidence->stringForm(), v_confidence},
					{reification::isUncertain->stringForm(),   v_uncertain},
					{reification::isOccasional->stringForm(),  v_occasional}
			});

			backend->query(reified, [&](const BindingsPtr &bindings) {
				FramedTripleView triple;
				if (q.instantiateInto(triple, bindings)) {
					if (bindings->contains(v_begin->name())) {
						auto &t_begin = bindings->get(v_begin->name());
						if (t_begin->isNumeric()) {
							triple.setBegin(std::static_pointer_cast<Numeric>(t_begin)->asDouble());
						}
					}
					if (bindings->contains(v_end->name())) {
						auto &t_end = bindings->get(v_end->name());
						if (t_end->isNumeric()) {
							triple.setEnd(std::static_pointer_cast<Numeric>(t_end)->asDouble());
						}
					}
					if (bindings->contains(v_confidence->name())) {
						auto &t_confidence = bindings->get(v_confidence->name());
						if (t_confidence->isNumeric()) {
							triple.setConfidence(std::static_pointer_cast<Numeric>(t_confidence)->asDouble());
						}
					}
					if (bindings->contains(v_uncertain->name())) {
						auto &t_uncertain = bindings->get(v_uncertain->name());
						if (t_uncertain->isNumeric()) {
							triple.setIsUncertain(std::static_pointer_cast<Boolean>(t_uncertain)->asBoolean());
						}
					} else {
						triple.setIsUncertain(false);
					}
					if (bindings->contains(v_occasional->name())) {
						auto &t_occasional = bindings->get(v_occasional->name());
						if (t_occasional->isNumeric()) {
							triple.setIsOccasional(std::static_pointer_cast<Boolean>(t_occasional)->asBoolean());
						}
					} else {
						triple.setIsOccasional(false);
					}
					visitor(triple);
				}
			});
		}
	}
}

void BackendInterface::query(const QueryableBackendPtr &backend, const GraphQueryPtr &q,
							 const BindingsHandler &callback) const {
	if (!backend->supports(BackendFeature::TripleContext) && ReifiedQuery::hasReifiablePattern(q)) {
		// if there is at least one reifiable pattern, we need to reify the query entirely,
		// and run the reified query on the original backend.
		auto reified = std::make_shared<ReifiedQuery>(q, vocabulary());
		backend->query(reified, [&](const BindingsPtr &bindings) {
			callback(bindings);
		});
	} else {
		backend->query(q, callback);
	}
}

void BackendInterface::pushIntoCursor(const QueryableBackendPtr &backend, const GraphPathQueryPtr &q,
									  const TokenBufferPtr &resultStream) const {
	auto expanded = backend->expand(q);
	auto channel = TokenStream::Channel::create(resultStream);
	try {
		bool hasPositiveAnswer = false;
		query(backend, expanded->expanded, [&](const BindingsPtr &bindings) {
			channel->push(backend->yes(expanded, bindings));
			hasPositiveAnswer = true;
		});
		if (!hasPositiveAnswer) {
			channel->push(backend->no(q));
		}
		channel->push(EndOfEvaluation::get());
	}
	catch (const std::exception &e) {
		// make sure EOS is pushed to the stream
		channel->push(EndOfEvaluation::get());
		throw;
	}
}

TokenBufferPtr BackendInterface::getAnswerCursor(const QueryableBackendPtr &backend, const GraphPathQueryPtr &q) {
	std::shared_ptr<TokenBuffer> result = std::make_shared<TokenBuffer>();
	auto runner =
			std::make_shared<ThreadPool::LambdaRunner>(
					[this, q, result, backend](const ThreadPool::LambdaRunner::StopChecker &) {
						pushIntoCursor(backend, q, result);
					});
	DefaultThreadPool()->pushWork(runner, [result, q](const std::exception &e) {
		KB_WARN("an exception occurred for graph query ({}): {}.", *q, e.what());
		result->close();
	});
	return result;
}
