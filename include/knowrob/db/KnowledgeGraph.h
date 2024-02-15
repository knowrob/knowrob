/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWLEDGE_GRAPH_H
#define KNOWROB_KNOWLEDGE_GRAPH_H

#include <boost/property_tree/ptree.hpp>
#include "memory"
#include "optional"
#include "raptor2.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/queries/TokenBuffer.h"
#include "knowrob/queries/ConjunctiveQuery.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/db/DataBackend.h"

namespace knowrob {
	/**
	 * A data structure that organizes statements in a graph.
	 * This is an abstract class with some virtual methods.
	 * It is supposed to be implemented by different backends
	 * that can be used to handle graph data.
	 */
	class KnowledgeGraph : public DataBackend {
	public:
		explicit KnowledgeGraph();

		KnowledgeGraph(const KnowledgeGraph &) = delete;

		~KnowledgeGraph();

		/**
		 * @param iri a RDF resource IRI
		 * @return true if the KG contains a statement involving the resource IRI
		 */
		bool isDefinedResource(const std::string_view &iri);

		/**
		 * @param iri a RDF property IRI
		 * @return true if the KG contains a statement involving the property IRI
		 */
		bool isDefinedProperty(const std::string_view &iri);

		/**
		 * @param iri a RDF class IRI
		 * @return true if the KG contains a statement involving the class IRI
		 */
		bool isDefinedClass(const std::string_view &iri);

		/**
		 * Submits a graph query to this knowledge graph.
		 * The query is evaluated concurrently, and evaluation may still be active
		 * when this function returns.
		 * The function returns a stream of solutions, the end of the stream is indicated
		 * by an EOS message.
		 * @param query a graph query
		 * @return a stream with answers to the query
		 */
		TokenBufferPtr submitQuery(const ConjunctiveQueryPtr &query);

		/**
		 * Evaluates a query and may block until evaluation completed.
		 * All results will be written into the provided stream object.
		 * @param query a query.
		 * @param resultStream a stream of answers.
		 */
		virtual void evaluateQuery(const ConjunctiveQueryPtr &query, const TokenBufferPtr &resultStream) = 0;

		virtual std::optional<std::string> getVersionOfOrigin(std::string_view origin) = 0;

		virtual void setVersionOfOrigin(std::string_view origin, std::string_view version) = 0;

	protected:
		std::shared_ptr<semweb::Vocabulary> vocabulary_;
		std::shared_ptr<semweb::ImportHierarchy> importHierarchy_;
	};

	using KnowledgeGraphPtr = std::shared_ptr<KnowledgeGraph>;

} // knowrob

#endif //KNOWROB_KNOWLEDGE_GRAPH_H
