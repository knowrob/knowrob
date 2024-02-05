//
// Created by daniel on 04.04.23.
//

#ifndef KNOWROB_MONGO_TRIPLE_LOADER_H
#define KNOWROB_MONGO_TRIPLE_LOADER_H

#include "string"
#include "memory"
#include "list"
#include "set"
#include "knowrob/backend/KnowledgeGraph.h"
#include "knowrob/semweb/Class.h"
#include "knowrob/semweb/Property.h"
#include "knowrob/semweb/Vocabulary.h"
#include "Collection.h"

namespace knowrob::mongo {
    /**
     * Handles loading of triples into the database.
     */
    class TripleLoader : public ITripleLoader {
    public:
        using ClassPair    = std::pair<semweb::ClassPtr, semweb::ClassPtr>;
        using PropertyPair = std::pair<semweb::PropertyPtr, semweb::PropertyPtr>;

        TripleLoader(std::string graphName,
                     const std::shared_ptr<Collection> &collection,
                     const std::shared_ptr<Collection> &oneCollection,
                     const semweb::VocabularyPtr &vocabulary,
                     uint32_t batchSize=1000);

        TripleLoader(const TripleLoader&) = delete;

        ~TripleLoader();

        /**
         * @return the named graph into which triples are loaded
         */
        const auto& graphName() const { return graphName_; }

        /**
         * @return list of import statements that occurred while loading
         */
        const auto& imports() const { return imports_; }

        /**
         * @return list of subclass statements that occurred while loading
         */
        const auto& subClassAssertions() const { return subClassAssertions_; }

        /**
         * @return list of subproperty statements that occurred while loading
         */
        const auto& subPropertyAssertions() const { return subPropertyAssertions_; }

        // Override ITripleLoader
        void loadTriple(const StatementData &tripleData) override;

        // Override ITripleLoader
        void flush() override;

    protected:
        const std::string graphName_;
        const uint32_t batchSize_;
        uint32_t operationCounter_;

        std::shared_ptr<Collection> tripleCollection_;
        std::shared_ptr<Collection> oneCollection_;
        std::shared_ptr<BulkOperation> bulkOperation_;
        semweb::VocabularyPtr vocabulary_;

        std::list<std::string> imports_;

        std::list<ClassPair> subClassAssertions_;
        std::list<PropertyPair> subPropertyAssertions_;

        bson_t* createTripleDocument(const StatementData &tripleData,
                                     const std::string &graphName,
                                     bool isTaxonomic);
    };

} // knowrob::mongo

#endif //KNOWROB_MONGO_TRIPLE_LOADER_H
