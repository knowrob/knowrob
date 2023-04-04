//
// Created by daniel on 04.04.23.
//

#ifndef KNOWROB_MONGO_TRIPLE_LOADER_H
#define KNOWROB_MONGO_TRIPLE_LOADER_H

#include "string"
#include "memory"
#include "list"
#include "set"
#include "knowrob/graphs/KnowledgeGraph.h"
#include "MongoCollection.h"

namespace knowrob {

    class MongoTripleLoader : public TripleLoader {
    public:
        MongoTripleLoader(std::string graphName,
                          const std::shared_ptr<MongoCollection> &collection,
                          const std::shared_ptr<MongoCollection> &oneCollection,
                          uint32_t batchSize=1000);

        const auto& imports() const { return imports_; }

        // Override TripleLoader
        void loadTriple(const TripleData &tripleData) override;
        void flush() override;
        void finish() override;

    protected:
        const std::string graphName_;
        const uint32_t batchSize_;
        uint32_t operationCounter_;

        std::shared_ptr<MongoCollection> tripleCollection_;
        std::shared_ptr<MongoCollection> oneCollection_;
        std::shared_ptr<MongoBulkOperation> bulkOperation_;

        std::list<std::string> imports_;
        std::list<std::pair<const char*,const char*>> subClassAssertions_;
        std::list<std::pair<const char*,const char*>> subPropertyAssertions_;

        bson_decimal128_t timeZero_;
        bson_decimal128_t timeInfinity_;

        static std::set<std::string_view> annotationProperties_;
        static bool hasStaticInitialization_;

        bson_t* createTripleDocument(const TripleData &tripleData,
                                     const std::string &graphName,
                                     bool isTaxonomic);
    };

} // knowrob

#endif //KNOWROB_MONGO_TRIPLE_LOADER_H
