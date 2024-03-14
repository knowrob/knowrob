/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REIFIED_QUERYABLE_BACKEND_H
#define KNOWROB_REIFIED_QUERYABLE_BACKEND_H

#include "knowrob/db/QueryableBackend.h"

namespace knowrob {
	/**
	 * A backend that reifies queries before forwarding them to another backend.
	 * Note that assertion and retraction of reified statements is not handled here,
	 * as this is done via transactions in the KB to avoid redundant reification.
	 */
	class ReifiedBackend : public QueryableBackend {
	public:
		explicit ReifiedBackend(QueryableBackendPtr backend);

		// Inherited via DataBackend
		bool initializeBackend(const ReasonerConfig &config) override;

		// Inherited via DataBackend
		bool canStoreTripleContext() const override;

		// Inherited via DataBackend
		bool insertOne(const FramedTriple &triple) override;

		// Inherited via DataBackend
		bool insertAll(const semweb::TripleContainerPtr &triples) override;

		// Inherited via DataBackend
		bool removeOne(const FramedTriple &triple) override;

		// Inherited via DataBackend
		bool removeAll(const semweb::TripleContainerPtr &triples) override;

		// Inherited via DataBackend
		bool removeAllWithOrigin(std::string_view origin) override;

		// Inherited via QueryableBackend
		bool isPersistent() const override { return originalBackend_->isPersistent(); }

		// Inherited via QueryableBackend
		bool contains(const FramedTriple &triple) override;

		// Inherited via QueryableBackend
		void foreach(const semweb::TripleVisitor &visitor) const override;

		// Inherited via QueryableBackend
		void batch(const semweb::TripleHandler &callback) const override;

		// Inherited via QueryableBackend
		void match(const FramedTriplePattern &query, const semweb::TripleVisitor &visitor) override;

		// Inherited via QueryableBackend
		void query(const GraphQueryPtr &query, const BindingsHandler &callback) override;

		// Inherited via QueryableBackend
		void count(const ResourceCounter &callback) const override;

	protected:
		QueryableBackendPtr originalBackend_;
	};

} // knowrob

#endif //KNOWROB_REIFIED_QUERYABLE_BACKEND_H
