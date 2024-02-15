/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_BACKEND_H
#define KNOWROB_PROLOG_BACKEND_H

#include "knowrob/db/KnowledgeGraph.h"
#include "PrologTerm.h"

//#define KNOWROB_USE_PROLOG_RDF11
#define KNOWROB_USE_PROLOG_RDF_DB

namespace knowrob {
	/**
	 * The triple store backend for Prolog reasoners.
	 * The backend is a singleton, and it is not possible to have multiple instances of it.
	 * The reason is that Prolog reasoners do not support multiple EDBs due to limitations of the
	 * underlying Prolog "semweb" library which only has a global storage.
	 */
	class PrologBackend : public KnowledgeGraph {
	public:
		// override DataBackend
		bool loadConfig(const ReasonerConfig &cfg) override;

		// override DataBackend
		bool insertOne(const StatementData &triple) override;

		// override DataBackend
		bool insertAll(const semweb::TripleContainerPtr &triples) override;

		// override DataBackend
		bool removeOne(const StatementData &triple) override;

		// override DataBackend
		bool removeAll(const semweb::TripleContainerPtr &triples) override;

		// override DataBackend
		bool removeAllWithOrigin(std::string_view origin) override;

		// override DataBackend
		bool removeAllMatching(const RDFLiteral &query) override;

		// override KnowledgeGraph
		void evaluateQuery(const ConjunctiveQueryPtr &query, const TokenBufferPtr &resultStream) override;

		// override KnowledgeGraph
		std::optional<std::string> getVersionOfOrigin(std::string_view origin) override;

		// override KnowledgeGraph
		void setVersionOfOrigin(std::string_view origin, std::string_view version) override;

	protected:
		static PrologTerm transaction(std::string_view rdf_functor, const semweb::TripleContainerPtr &triples);
	};

} // knowrob

#endif //KNOWROB_PROLOG_BACKEND_H
