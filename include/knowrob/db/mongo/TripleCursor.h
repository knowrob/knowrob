//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_MONGO_TRIPLE_CURSOR_H
#define KNOWROB_MONGO_TRIPLE_CURSOR_H

#include "Cursor.h"
#include "knowrob/db/KnowledgeGraph.h"

namespace knowrob::mongo {
    /**
     * A cursor that iterates over different results of a query,
     * and fills a StatementData structure with the data.
     */
    class TripleCursor : public Cursor {
    public:
        explicit TripleCursor(const std::shared_ptr<Collection> &collection);

        /**
         * Get the next triple from this cursor if any.
         * @param tripleData a triple data structure
         * @return true on success
         */
        bool nextTriple(StatementData &tripleData);

        /**
         * @return the last document fetched by this cursor, or null if no document was fetched before.
         */
        auto tripleDocument() { return tripleDocument_; }

    protected:
        const bson_t *tripleDocument_;
        bson_iter_t tripleIter_;
    };

} // knowrob

#endif //KNOWROB_MONGO_TRIPLE_CURSOR_H
