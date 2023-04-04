//
// Created by daniel on 01.04.23.
//

#ifndef KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
#define KNOWROB_MONGO_KNOWLEDGE_GRAPH_H

#include <optional>
#include <list>
#include "boost/property_tree/ptree.hpp"
#include "knowrob/graphs/KnowledgeGraph.h"
#include "MongoCollection.h"

namespace knowrob {
    class MongoKnowledgeGraph : public KnowledgeGraph {
    public:
        explicit MongoKnowledgeGraph(const char* db_uri="mongodb://localhost:27017",
                                     const char* db_name="knowrob",
                                     const char* collectionName="triples");

        explicit MongoKnowledgeGraph(const boost::property_tree::ptree &config);

        void drop();

        void dropGraph(const std::string &graphName);

        std::optional<std::string> getCurrentGraphVersion(const std::string &graphName);

        // Override KnowledgeGraph
        bool loadTriples(const std::string &uriString, TripleFormat format) override;

    protected:
        std::shared_ptr<MongoCollection> tripleCollection_;
        std::shared_ptr<MongoCollection> oneCollection_;

        void initialize();

        void createSearchIndices();

        void setCurrentGraphVersion(const std::string &graphName,
                                    const std::string &graphURI,
                                    const std::string &graphVersion);

        static const char* getDBName(const boost::property_tree::ptree &config);

        static const char* getCollectionName(const boost::property_tree::ptree &config);

        static std::string getURI(const boost::property_tree::ptree &config);
    };

} // knowrob

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
