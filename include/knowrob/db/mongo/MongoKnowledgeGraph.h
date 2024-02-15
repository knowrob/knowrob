//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
#define KNOWROB_MONGO_KNOWLEDGE_GRAPH_H

#include <optional>
#include <list>
#include "boost/property_tree/ptree.hpp"
#include "knowrob/db/KnowledgeGraph.h"
#include "knowrob/db/mongo/Collection.h"
#include "knowrob/queries/TokenBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/db/mongo/AnswerCursor.h"
#include "knowrob/semweb/ImportHierarchy.h"

namespace knowrob {
	/**
	 * A knowledge graph implemented with MongoDB.
	 */
	class MongoKnowledgeGraph : public knowrob::KnowledgeGraph {
	public:
		MongoKnowledgeGraph();

		/**
		 * Constructor with configuration.
		 * There is no need to call loadConfig if this constructor is used.
		 * @param db_uri MongoDB URI string
		 * @param db_name MongoDB database name where KG is stored
		 * @param collectionName MongoDB collection name where KG is stored
		 */
		explicit MongoKnowledgeGraph(
				const char *db_uri,
				const char *db_name = "knowrob",
				const char *collectionName = "triples");

		/**
		 * @return the name of the database.
		 */
		const std::string &dbName() const;

		/**
		 * @return the URI string used to connect to the database.
		 */
		const std::string &dbURI() const;

		/**
		 * @return true if only read operations are allowed.
		 */
		bool isReadOnly() const;

		/**
		 * (re)create search indices.
		 */
		void createSearchIndices();

		/**
		 * Delete all statements in the database.
		 * Note: ths will also delete all indices which need to be re-created afterwards.
		 */
		void drop();

		/**
		 * Lookup up all matching triples.
		 * @param tripleExpression a triple expression
		 * @return a cursor over matching triples
		 */
		mongo::AnswerCursorPtr lookup(const RDFLiteral &tripleExpression);

		/**
		 * Lookup up all matching triples.
		 * @param tripleData an atomic proposition
		 * @return a cursor over matching triples
		 */
		mongo::AnswerCursorPtr lookup(const StatementData &tripleData);

		/**
		 * Lookup up a path of matching triples.
		 * The lookup pipeline includes a step for each expression in the vector
		 * in the same order as the expressions are ordered in the vector.
		 * @param tripleExpressions a vector of triple expressions
		 * @return a cursor over matching triples
		 */
		mongo::AnswerCursorPtr lookup(const std::vector<RDFLiteralPtr> &tripleExpressions, uint32_t limit = 0);

		/**
		 * Watch for instantiations of a literal in the knowledge graph.
		 * @param literal a literal
		 * @return a stream with answers to the query
		 */
		TokenBufferPtr watchQuery(const ConjunctiveQueryPtr &literal);

		// Override KnowledgeGraph
		bool loadConfig(const ReasonerConfig &config) override;

		// Override KnowledgeGraph
		std::optional<std::string> getVersionOfOrigin(std::string_view origin) override;

		// Override KnowledgeGraph
		void setVersionOfOrigin(std::string_view origin, std::string_view version) override;

		// Override IDataBackend
		bool insertOne(const StatementData &triple) override;

		// Override IDataBackend
		bool insertAll(const semweb::TripleContainerPtr &triples) override;

		// Override IDataBackend
		bool removeOne(const StatementData &triple) override;

		// Override IDataBackend
		bool removeAll(const semweb::TripleContainerPtr &triples) override;

		// Override IDataBackend
		bool removeAllWithOrigin(std::string_view origin) override;

		// Override IDataBackend
		bool removeAllMatching(const RDFLiteral &query) override;

		// Override KnowledgeGraph
		void evaluateQuery(const ConjunctiveQueryPtr &query, const TokenBufferPtr &resultStream) override;

	protected:
		using StringPair = std::pair<std::string_view, std::string_view>;

		std::shared_ptr<mongo::Collection> tripleCollection_;
		std::shared_ptr<mongo::Collection> oneCollection_;
		bool isReadOnly_;

		void initialize();

		static std::shared_ptr<mongo::Collection> connect(const boost::property_tree::ptree &config);

		static std::string getDBName(const boost::property_tree::ptree &config);

		static std::string getCollectionName(const boost::property_tree::ptree &config);

		bson_t *createTripleDocument(const StatementData &tripleData, const std::string &graphName, bool isTaxonomic);

		static std::string getURI(const boost::property_tree::ptree &config);

		void updateHierarchy(
				const std::vector<StringPair> &subClassAssertions,
				const std::vector<StringPair> &subPropertyAssertions);

		void updateTimeInterval(const StatementData &tripleLoader);

		static bson_t *getSelector(const RDFLiteral &tripleExpression, bool isTaxonomicProperty);

		static bson_t *getSelector(const StatementData &triple, bool isTaxonomicProperty);

		bool isTaxonomicProperty(const TermPtr &propertyTerm);

		bool isTaxonomicProperty(const char *property);

		bool dropOrigin(std::string_view origin);

		friend class MongoKnowledgeGraphTest;
	};

} // knowrob::mongo

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
