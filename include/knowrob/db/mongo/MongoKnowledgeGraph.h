//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
#define KNOWROB_MONGO_KNOWLEDGE_GRAPH_H

#include <optional>
#include <list>
#include "boost/property_tree/ptree.hpp"
#include "knowrob/db/DataBackend.h"
#include "knowrob/db/QueryableBackend.h"
#include "knowrob/db/PersistentBackend.h"
#include "knowrob/db/mongo/Collection.h"
#include "knowrob/queries/TokenBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/db/mongo/AnswerCursor.h"
#include "knowrob/semweb/ImportHierarchy.h"

namespace knowrob {
	/**
	 * A knowledge graph implemented with MongoDB.
	 */
	class MongoKnowledgeGraph : public DataBackend,
			                    public QueryableBackend,
								public PersistentBackend {
	public:
		static const std::string DB_URI_DEFAULT;
		static const std::string DB_NAME_KNOWROB;
		static const std::string DB_NAME_TESTS;
		static const std::string COLL_NAME_TRIPLES;
		static const std::string COLL_NAME_TESTS;

		MongoKnowledgeGraph();

		/**
		 * Initialize the knowledge graph with a MongoDB URI.
		 * @param db_uri the URI string used to connect to the database.
		 * @param db_name the name of the database.
		 * @param collectionName the name of the collection for triples.
		 * @return true on success
		 */
		bool init(std::string_view db_uri, std::string_view db_name = "knowrob", std::string_view collectionName = "triples");

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
		bool initializeBackend(const ReasonerConfig &config) override;

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

		// Override PersistentBackend
		std::optional<std::string> getVersionOfOrigin(std::string_view origin) override;

		// Override PersistentBackend
		void setVersionOfOrigin(std::string_view origin, std::string_view version) override;

		// Override QueryableBackend
		void evaluateQuery(const ConjunctiveQueryPtr &query, const TokenBufferPtr &resultStream) override;

	protected:
		using StringPair = std::pair<std::string_view, std::string_view>;

		std::shared_ptr<mongo::Collection> tripleCollection_;
		std::shared_ptr<mongo::Collection> oneCollection_;
		bool isReadOnly_;

		void initialize();

		static std::shared_ptr<mongo::Collection> connect(const boost::property_tree::ptree &config);

		static std::shared_ptr<mongo::Collection> connect(std::string_view db_uri, std::string_view db_name, std::string_view collectionName);

		static std::string getDBName(const boost::property_tree::ptree &config);

		static std::string getCollectionName(const boost::property_tree::ptree &config);

		bson_t *createTripleDocument(const StatementData &tripleData, const std::string &graphName, bool isTaxonomic);

		static std::string getURI(const boost::property_tree::ptree &config);

		void updateHierarchy(
				const std::vector<StringPair> &subClassAssertions,
				const std::vector<StringPair> &subPropertyAssertions);

		void updateTimeInterval(const StatementData &tripleLoader);

		bson_t *getSelector(const RDFLiteral &tripleExpression, bool isTaxonomicProperty);

		bson_t *getSelector(const StatementData &triple, bool isTaxonomicProperty);

		bool isTaxonomicProperty(const TermPtr &propertyTerm);

		bool isTaxonomicProperty(const char *property);

		bool dropOrigin(std::string_view origin);

		bool dropSessionOrigins();

		friend class MongoKnowledgeGraphTest;
	};

} // knowrob::mongo

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
