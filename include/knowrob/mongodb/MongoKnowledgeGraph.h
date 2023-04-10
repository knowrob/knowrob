//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
#define KNOWROB_MONGO_KNOWLEDGE_GRAPH_H

#include <optional>
#include <list>
#include "boost/property_tree/ptree.hpp"
#include "knowrob/semweb/KnowledgeGraph.h"
#include "knowrob/semweb/Vocabulary.h"
#include "Collection.h"
#include "Cursor.h"
#include "knowrob/queries/BufferedAnswerStream.h"
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
        explicit MongoKnowledgeGraph(const char* db_uri="mongodb://localhost:27017",
                                     const char* db_name="knowrob",
                                     const char* collectionName="triples");

        explicit MongoKnowledgeGraph(const boost::property_tree::ptree &config);

        const auto& importHierarchy() const { return importHierarchy_; }

        mongo::AnswerCursorPtr lookupTriples(const semweb::TripleExpression &tripleExpression);

        mongo::AnswerCursorPtr lookupTriplePaths(const std::vector<semweb::TripleExpression> &tripleExpressions);

        /**
         * (re)create search indices.
         */
        void createSearchIndices();

        /**
         * Delete all triples of a named graph in this knowledge graph.
         * @param graphName a graph name
         */
        void dropGraph(const std::string &graphName);

        /**
         * Delete all triples in the database.
         * Note: ths will also delete all indices which need to be re-created afterwards.
         */
        void drop();

        /**
         * @param graphName the name of a graph
         * @return the version string associated to the named graph if any
         */
        std::optional<std::string> getCurrentGraphVersion(const std::string &graphName);

        // Override KnowledgeGraph
        void assertTriple(const TripleData &tripleData) override;

        // Override KnowledgeGraph
        void assertTriples(const std::vector<TripleData> &tripleData) override;

        // Override KnowledgeGraph
        void removeAllTriples(const semweb::TripleExpression &tripleExpression) override;

        // Override KnowledgeGraph
        void removeOneTriple(const semweb::TripleExpression &tripleExpression) override;

        // Override KnowledgeGraph
        BufferedAnswerStreamPtr submitQuery(const GraphQueryPtr &literal) override;

        // Override KnowledgeGraph
        BufferedAnswerStreamPtr watchQuery(const GraphQueryPtr &literal) override;

        // Override KnowledgeGraph
        bool loadTriples(const std::string_view &uriString, TripleFormat format) override;

    protected:
        std::shared_ptr<mongo::Collection> tripleCollection_;
        std::shared_ptr<mongo::Collection> oneCollection_;
        std::shared_ptr<semweb::ImportHierarchy> importHierarchy_;

        void initialize();

        void setCurrentGraphVersion(const std::string &graphName,
                                    const std::string &graphURI,
                                    const std::string &graphVersion);

        static const char* getDBName(const boost::property_tree::ptree &config);

        static const char* getCollectionName(const boost::property_tree::ptree &config);

        static std::string getURI(const boost::property_tree::ptree &config);

        void updateHierarchy(mongo::TripleLoader &tripleLoader);

        static bson_t* getTripleSelector(const semweb::TripleExpression &tripleExpression,
                                  bool isTaxonomicProperty);

        bool isTaxonomicProperty(const TermPtr &propertyTerm);
    };

} // knowrob::mongo

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
