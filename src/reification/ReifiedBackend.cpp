/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reification/ReifiedBackend.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/UnReificationContainer.h"
#include "knowrob/reification/ReifiedQuery.h"

using namespace knowrob;

ReifiedBackend::ReifiedBackend(QueryableBackendPtr backend)
		: originalBackend_(std::move(backend)) {
}

bool ReifiedBackend::initializeBackend(const ReasonerConfig &config) {
	return originalBackend_->initializeBackend(config);
}

bool ReifiedBackend::canStoreTripleContext() const {
	return originalBackend_->canStoreTripleContext();
}

bool ReifiedBackend::insertOne(const FramedTriple &triple) {
	return originalBackend_->insertOne(triple);
}

bool ReifiedBackend::insertAll(const semweb::TripleContainerPtr &triples) {
	return originalBackend_->insertAll(triples);
}

bool ReifiedBackend::removeOne(const FramedTriple &triple) {
	return originalBackend_->removeOne(triple);
}

bool ReifiedBackend::removeAll(const semweb::TripleContainerPtr &triples) {
	return originalBackend_->removeAll(triples);
}

bool ReifiedBackend::removeAllWithOrigin(std::string_view origin) {
	return originalBackend_->removeAllWithOrigin(origin);
}

bool ReifiedBackend::contains(const FramedTriple &triple) {
	ReifiedTriple reification(triple, vocabulary());
	bool containsAll = false;
	for (auto &reified: reification) {
		containsAll = originalBackend_->contains(*reified.ptr);
		if (!containsAll) {
			break;
		}
	}
	return containsAll;
}

void ReifiedBackend::foreach(const semweb::TripleVisitor &visitor) const {
	// fill a container that reverses a reification.
	UnReificationContainer unReifiedTriples;
	// the UnReificationContainer uses triple views, but memory of original triples can be
	// lost in the loop. So we need to store the original triples in a vector, and create
	// a view on them in the UnReificationContainer.
	// TODO: copy of original triples can be avoided by switching to FramedTriplePtr and taking over ownership in the loop.
	std::vector<FramedTripleCopy> originalTriples;
	// finally loop over all original triples
	originalBackend_->foreach([&](const FramedTriple &triple) {
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

void ReifiedBackend::batch(const semweb::TripleHandler &callback) const {
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
	originalBackend_->batch([&](const semweb::TripleContainerPtr &triples) {
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

void ReifiedBackend::match(const FramedTriplePattern &q, const semweb::TripleVisitor &visitor) {
	static auto ctx = std::make_shared<QueryContext>();
	auto flags = ReifiedQuery::getReificationFlags(q);
	if (flags & IncludeOriginal) {
		originalBackend_->match(q, visitor);
	}
	if (flags & IncludeReified) {
		auto reified = std::make_shared<ReifiedQuery>(q, vocabulary());
		originalBackend_->query(reified, [&](const BindingsPtr &bindings) {
			FramedTripleView triple;
			if (q.instantiateInto(triple, bindings)) {
				visitor(triple);
			}
		});
	}
}

void ReifiedBackend::query(const GraphQueryPtr &q, const BindingsHandler &callback) {
	if (ReifiedQuery::hasReifiablePattern(q)) {
		// if there is at least one reifiable pattern, we need to reify the query entirely,
		// and run the reified query on the original backend.
		auto reified = std::make_shared<ReifiedQuery>(q, vocabulary());
		originalBackend_->query(reified, [&](const BindingsPtr &bindings) {
			callback(bindings);
		});
	} else {
		originalBackend_->query(q, callback);
	}
}

void ReifiedBackend::count(const ResourceCounter &callback) const {
	// TODO: implement
	originalBackend_->count(callback);
}
