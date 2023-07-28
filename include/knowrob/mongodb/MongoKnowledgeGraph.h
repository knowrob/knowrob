//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
#define KNOWROB_MONGO_KNOWLEDGE_GRAPH_H

#include <optional>
#include <list>
#include "boost/property_tree/ptree.hpp"
#include "knowrob/backend/KnowledgeGraph.h"
#include "knowrob/mongodb/Collection.h"
#include "knowrob/queries/AnswerBuffer.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/mongodb/TripleLoader.h"
#include "knowrob/mongodb/AnswerCursor.h"
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
         * There is no need to call loadConfiguration if this constructor is used.
         * @param db_uri MongoDB URI string
         * @param db_name MongoDB database name where KG is stored
         * @param collectionName MongoDB collection name where KG is stored
         */
        explicit MongoKnowledgeGraph(
                const char* db_uri,
                const char* db_name="knowrob",
                const char* collectionName="triples");

        /**
         * (re)create search indices.
         */
        void createSearchIndices();

        /**
         * Delete all statements in a named graph
         * @param graphName a graph name
         */
        void dropGraph(const std::string_view &graphName);

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
        mongo::AnswerCursorPtr lookup(const semweb::FramedLiteral &tripleExpression);

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
        mongo::AnswerCursorPtr lookup(const std::list<semweb::FramedLiteral> &tripleExpressions);

        /**
         * @param graphName the name of a graph
         * @return the version string associated to the named graph if any
         */
        std::optional<std::string> getCurrentGraphVersion(const std::string &graphName);

        // Override KnowledgeGraph
        bool loadConfiguration(const boost::property_tree::ptree &config) override;

        // Override KnowledgeGraph
        bool loadFile(const std::string_view &uriString, TripleFormat format, const ModalityFrame &frame) override;

        // Override KnowledgeGraph
        bool insert(const StatementData &tripleData) override;

        // Override KnowledgeGraph
        bool insert(const std::vector<StatementData> &tripleData) override;

        // Override KnowledgeGraph
        void removeAll(const semweb::FramedLiteral &tripleExpression) override;

        // Override KnowledgeGraph
        void removeOne(const semweb::FramedLiteral &tripleExpression) override;

        // Override KnowledgeGraph
        void evaluateQuery(const GraphQueryPtr &query, AnswerBufferPtr &resultStream) override;

        // Override KnowledgeGraph
        AnswerBufferPtr watchQuery(const GraphQueryPtr &literal) override;

    protected:
        std::shared_ptr<mongo::Collection> tripleCollection_;
        std::shared_ptr<mongo::Collection> oneCollection_;

        void initialize();

        void setCurrentGraphVersion(const std::string &graphName,
                                    const std::string &graphURI,
                                    const std::string &graphVersion);

        static std::shared_ptr<mongo::Collection> connect(const boost::property_tree::ptree &config);

        static const char* getDBName(const boost::property_tree::ptree &config);

        static const char* getCollectionName(const boost::property_tree::ptree &config);

        static std::string getURI(const boost::property_tree::ptree &config);

        void updateHierarchy(mongo::TripleLoader &tripleLoader);

        void updateTimeInterval(const StatementData &tripleLoader);

        static bson_t* getSelector(const semweb::FramedLiteral &tripleExpression, bool isTaxonomicProperty);

        bool isTaxonomicProperty(const TermPtr &propertyTerm);
    };

} // knowrob::mongo

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
