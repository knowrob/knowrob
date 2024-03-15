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

using namespace knowrob;

std::shared_ptr<transaction::Transaction> BackendInterface::createTransaction(
		TransactionType transactionType,
		BackendSelection transactionTargets,
		const std::vector<std::shared_ptr<DefinedBackend>> &backends) {
	std::shared_ptr<transaction::Transaction> transaction;
	if (transactionType == Insert) {
		transaction = std::make_shared<transaction::Insert>(vocabulary(), importHierarchy());
	} else {
		transaction = std::make_shared<transaction::Remove>(vocabulary(), importHierarchy());
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
	if (!importHierarchy()->isReservedOrigin(origin)) {
		importHierarchy()->removeCurrentGraph(origin);
	}

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

void BackendInterface::foreach(const QueryableBackendPtr &backend, const semweb::TripleVisitor &visitor) const {
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

void BackendInterface::batch(const QueryableBackendPtr &backend, const semweb::TripleHandler &callback) const {
	if (backend->supports(BackendFeature::TripleContext)) {
		backend->batch(callback);
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
	backend->batch([&](const semweb::TripleContainerPtr &triples) {
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

void BackendInterface::match(const QueryableBackendPtr &backend, const FramedTriplePattern &q,
							 const semweb::TripleVisitor &visitor) const {
	static auto ctx = std::make_shared<QueryContext>();
	if (backend->supports(BackendFeature::TripleContext)) {
		backend->match(q, visitor);
	} else {
		auto flags = ReifiedQuery::getReificationFlags(q);
		if (flags & IncludeOriginal) {
			backend->match(q, visitor);
		}
		if (flags & IncludeReified) {
			auto reified = std::make_shared<ReifiedQuery>(q, vocabulary());
			backend->query(reified, [&](const BindingsPtr &bindings) {
				FramedTripleView triple;
				if (q.instantiateInto(triple, bindings)) {
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
