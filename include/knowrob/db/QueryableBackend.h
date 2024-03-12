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

namespace knowrob {
	using ResourceCounter = std::function<void(std::string_view, uint64_t)>;

	/**
	 * A backend that can be queried.
	 */
	class QueryableBackend : public DataBackend {
	public:
		static AtomPtr versionProperty;

		QueryableBackend();

		/**
		 * @return true if the backend is persistent.
		 */
		virtual bool isPersistent() const = 0;

		/**
		 * @param triple a framed triple.
		 * @return true if the model contains the triple.
		 */
		virtual bool contains(const FramedTriple &triple) = 0;

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
		 * @param query a framed triple pattern.
		 * @param handler a function that is called for each matching framed triple.
		 */
		virtual void match(const FramedTriplePattern &query, const semweb::TripleVisitor &visitor) = 0;

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
		 * Submits a graph query to this knowledge graph.
		 * The query is evaluated concurrently, and evaluation may still be active
		 * when this function returns.
		 * The function returns a stream of solutions, the end of the stream is indicated
		 * by an EOS message.
		 * @param query a graph query
		 * @return a stream with answers to the query
		 */
		TokenBufferPtr submitQuery(const GraphPathQueryPtr &query);

		/**
		 * Evaluates a query and may block until evaluation completed.
		 * All results will be written into the provided stream object.
		 * @param query a query.
		 * @param resultStream a stream of answers.
		 */
		void evaluateQuery(const GraphPathQueryPtr &query, const TokenBufferPtr &resultStream);

		/**
		 * @return a list of all origins that have been persisted.
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
		 * @return the batch size.
		 */
		uint32_t batchSize() const { return batchSize_; }

		/**
		 * The size of the container used when triples are processed in batches.
		 * @param batchSize the batch size for the backend.
		 */
		void setBatchSize(uint32_t batchSize) { batchSize_ = batchSize; }

		/**
		 * Expand a query with complex patterns into one with potentially more
		 * but less complex patterns.
		 * This is done to realize a backend-independent handling of some aspects
		 * such as determining whether an answer is uncertain or not.
		 * @param q a graph query.
		 * @return the expanded graph query.
		 */
		static GraphQueryPtr expand(const GraphQueryPtr &q);

	protected:
		uint32_t batchSize_;

		struct ExpansionContext {
			ExpansionContext() : counter(0) {}

			std::vector<VariablePtr> o_vars;
			std::vector<VariablePtr> u_vars;
			VariablePtr accumulated_begin;
			VariablePtr accumulated_end;
			QueryContextPtr query_ctx;
			uint32_t counter;
		};

		static std::shared_ptr<AnswerNo> no(const GraphPathQueryPtr &q);

		static std::shared_ptr<AnswerYes> yes(const GraphPathQueryPtr &q, const BindingsPtr &bindings);

		static GraphQueryPtr expand(ExpansionContext &ctx, const GraphQueryPtr &q);

		static std::shared_ptr<GraphTerm> expand(ExpansionContext &ctx, const std::shared_ptr<GraphTerm> &q);

		static std::shared_ptr<GraphTerm> expandPattern(ExpansionContext &ctx, const std::shared_ptr<GraphPattern> &q);

		static bool expandAll(ExpansionContext &ctx, const std::shared_ptr<GraphConnective> &q,
							  std::vector<std::shared_ptr<GraphTerm>> &expandedTerms);
	};

	using QueryableBackendPtr = std::shared_ptr<QueryableBackend>;
}

#endif //KNOWROB_QUERYABLE_BACKEND_H
