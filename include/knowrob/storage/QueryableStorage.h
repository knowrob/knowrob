/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERYABLE_STORAGE_H
#define KNOWROB_QUERYABLE_STORAGE_H

#include <knowrob/semweb/TripleFormat.h>

#include "knowrob/queries/TokenBuffer.h"
#include "Storage.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/semweb/GraphPathQuery.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/semweb/GraphConnective.h"
#include "knowrob/semweb/GraphQueryExpansion.h"
#include "VersionedOrigin.h"

namespace knowrob {
	using ResourceCounter = std::function<void(std::string_view, uint64_t)>;

	/**
	 * A backend that can be queried.
	 */
	class QueryableStorage : public Storage {
	public:
		static AtomPtr versionProperty;

		explicit QueryableStorage(StorageFeatures features = StorageFeature::NothingSpecial);

		~QueryableStorage() override = default;

		/**
		 * @return true if the backend is persistent.
		 */
		virtual bool isPersistent() const = 0;

		/**
		 * Iterate over all triples in the model.
		 * @param visitor the callback to handle the triples.
		 * @return true if the iteration was successful.
		 */
		virtual void foreach(const TripleVisitor &visitor) const;

		/**
		 * Iterate over all triples in the model.
		 * @param callback the callback to handle the triples.
		 * @return true if the iteration was successful.
		 */
		virtual void batch(const TripleHandler &callback) const = 0;

		/**
		 * Iterate over all triples in the model that have a given origin.
		 * @param origin the origin of the triples.
		 * @param callback the callback to handle the triples.
		 * @return true if the iteration was successful.
		 */
		virtual void batchOrigin(std::string_view origin, const TripleHandler &callback) = 0;

		/**
		 * @param triple a framed triple.
		 * @return true if the model contains the triple.
		 */
		virtual bool contains(const Triple &triple);

		/**
		 * @param query a framed triple pattern.
		 * @param visitor a function that is called for each matching framed triple.
		 */
		virtual void match(const TriplePattern &query, const TripleVisitor &visitor);

		/**
		 * Submits a graph query to this storage.
		 * @param query a graph query
		 * @param callback a function that is called for each answer to the query.
		 */
		virtual void query(const GraphQueryPtr &query, const BindingsHandler &callback) = 0;

		/**
		 * @param callback a function that is called for each resource and its count.
		 */
		virtual void count(const ResourceCounter &callback) const = 0;

		/**
		 * Export all triples in the model to a file.
		 * @param filename the name of the file to export to.
		 * @param format the format of the output file.
		 * @return true if the export was successful
		 */
		bool exportTo(const std::string &filename,
		              semweb::TripleFormat format=semweb::RDF_XML) const;

		/**
		 * @return a list of all origins that have been asserted.
		 */
		std::vector<VersionedOriginPtr> getOrigins();

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
		 * Delete triples that have been asserted from a "session" origin.
		 */
		void dropSessionOrigins();

		/**
		 * Compute the expansion of a graph path query.
		 * @param q a graph path query.
		 * @return the expansion of the query.
		 */
		GraphQueryExpansionPtr expand(const GraphQueryPtr &q);

		/**
		 * Generates a positive answer to a query.
		 * @param original a graph path query.
		 * @param expansion a graph query expansion.
		 * @param bindings a set of bindings.
		 * @return a positive answer to the query.
		 */
		static std::shared_ptr<AnswerYes> yes(const GraphPathQueryPtr &original,
											  const GraphQueryExpansionPtr &expansion,
											  const BindingsPtr &bindings);

		/**
		 * Generates a negative answer to a query.
		 * @param q a graph path query.
		 * @return a negative answer to the query.
		 */
		static std::shared_ptr<AnswerNo> no(const GraphPathQueryPtr &q);
	};

	using QueryableBackendPtr = std::shared_ptr<QueryableStorage>;
}

#endif //KNOWROB_QUERYABLE_STORAGE_H
