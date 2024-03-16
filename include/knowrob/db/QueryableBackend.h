/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERYABLE_BACKEND_H
#define KNOWROB_QUERYABLE_BACKEND_H

#include "knowrob/queries/TokenBuffer.h"
#include "DataBackend.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/triples/GraphPathQuery.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/triples/GraphConnective.h"
#include "knowrob/triples/GraphQueryExpansion.h"

namespace knowrob {
	using ResourceCounter = std::function<void(std::string_view, uint64_t)>;

	/**
	 * A backend that can be queried.
	 */
	class QueryableBackend : public DataBackend {
	public:
		static AtomPtr versionProperty;

		explicit QueryableBackend(BackendFeatures features = BackendFeature::NothingSpecial);

		/**
		 * @return true if the backend is persistent.
		 */
		virtual bool isPersistent() const = 0;

		/**
		 * Iterate over all triples in the model.
		 * @param callback the callback to handle the triples.
		 * @return true if the iteration was successful.
		 */
		virtual void foreach(const semweb::TripleVisitor &visitor) const = 0;

		/**
		 * Iterate over all triples in the model.
		 * @param callback the callback to handle the triples.
		 * @return true if the iteration was successful.
		 */
		virtual void batch(const semweb::TripleHandler &callback) const = 0;

		/**
		 * @param triple a framed triple.
		 * @return true if the model contains the triple.
		 */
		virtual bool contains(const FramedTriple &triple);

		/**
		 * @param query a framed triple pattern.
		 * @param handler a function that is called for each matching framed triple.
		 */
		virtual void match(const FramedTriplePattern &query, const semweb::TripleVisitor &visitor);

		/**
		 * Submits a graph query to this knowledge graph.
		 * @param query a graph query
		 * @param callback a function that is called for each answer to the query.
		 */
		virtual void query(const GraphQueryPtr &query, const BindingsHandler &callback) = 0;

		/**
		 * @param callback a function that is called for each resource and its count.
		 */
		virtual void count(const ResourceCounter &callback) const = 0;

		/**
		 * @return a list of all origins that have been asserted.
		 */
		std::vector<std::string> getOrigins();

		/**
		 * @param origin an origin string.
		 * @return the version of the origin, or an empty optional if the origin is unknown.
		 */
		std::optional<std::string> getVersionOfOrigin(std::string_view origin);

		/**
		 * Set the version of an origin.
		 * @param origin an origin string.
		 * @param version a version string.
		 */
		void setVersionOfOrigin(std::string_view origin, std::string_view version);

		/**
		 * Compute the expansion of a graph path query.
		 * @param q a graph path query.
		 * @return the expansion of the query.
		 */
		GraphQueryExpansionPtr expand(const GraphPathQueryPtr &q);

		/**
		 * Generates a positive answer to a query.
		 * @param expansion a graph query expansion.
		 * @param bindings a set of bindings.
		 * @return a positive answer to the query.
		 */
		static std::shared_ptr<AnswerYes> yes(const GraphQueryExpansionPtr &expansion, const BindingsPtr &bindings);

		/**
		 * Generates a negative answer to a query.
		 * @param q a graph path query.
		 * @return a negative answer to the query.
		 */
		static std::shared_ptr<AnswerNo> no(const GraphPathQueryPtr &q);
	};

	using QueryableBackendPtr = std::shared_ptr<QueryableBackend>;
}

#endif //KNOWROB_QUERYABLE_BACKEND_H
