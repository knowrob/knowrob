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

    class MongoTripleLoader : public TripleLoader {
    public:
        MongoTripleLoader(std::string graphName,
                          const std::shared_ptr<MongoCollection> &collection,
                          uint32_t batchSize=1000);

        const auto& imports() const { return imports_; }

        const auto& subClassAssertions() const { return subClassAssertions_; }
        const auto& subPropertyAssertions() const { return subPropertyAssertions_; }

        // Override TripleLoader
        void loadTriple(const TripleData &tripleData) override;
        void flush() override;

    protected:
        const std::string graphName_;
        const uint32_t batchSize_;
        uint32_t operationCounter_;

        std::shared_ptr<MongoCollection> tripleCollection_;
        std::shared_ptr<MongoBulkOperation> bulkOperation_;

        std::list<std::string> imports_;
        std::list<std::pair<std::string,std::string>> subClassAssertions_;
        std::list<std::pair<std::string,std::string>> subPropertyAssertions_;

        bson_decimal128_t timeZero_;
        bson_decimal128_t timeInfinity_;

        static std::set<std::string_view> annotationProperties_;
        static bool hasStaticInitialization_;

        static std::string getSubjectString(const TripleData &tripleData, const std::string &graphName);
        static std::string getObjectString(const TripleData &tripleData, const std::string &graphName);

        bson_t* createTripleDocument(const TripleData &tripleData,
                                     const std::string &graphName,
                                     bool isTaxonomic);
    };

    class MongoKnowledgeGraph : public KnowledgeGraph {
    public:
        explicit MongoKnowledgeGraph(const char* db_uri="mongodb://localhost:27017",
                                     const char* db_name="knowrob",
                                     const char* collectionName="triples");

        explicit MongoKnowledgeGraph(const boost::property_tree::ptree &config);

        bool loadTriples(const std::string &uriString, TripleFormat format);

        void drop();

        void dropGraph(const std::string &graphName);

        std::optional<std::string> getCurrentGraphVersion(const std::string &graphName);

    protected:
        std::shared_ptr<MongoCollection> tripleCollection_;
        std::shared_ptr<MongoCollection> oneCollection_;

        void initialize();
        void createSearchIndices();

        void setCurrentGraphVersion(const std::string &graphURI,
                                    const std::string &graphVersion,
                                    const std::string &graphName);

        void updateGraphHierarchy(MongoTripleLoader &loader);
    };

} // knowrob

#endif //KNOWROB_MONGO_KNOWLEDGE_GRAPH_H
