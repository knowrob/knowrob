//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_MONGO_TRIPLE_CURSOR_H
#define KNOWROB_MONGO_TRIPLE_CURSOR_H

#include "Cursor.h"
#include "knowrob/semweb/KnowledgeGraph.h"

namespace knowrob::mongo {

    class TripleCursor : public Cursor {
    public:
        explicit TripleCursor(const std::shared_ptr<Collection> &collection);

        bool nextTriple(TripleData &tripleData);

    protected:
        const bson_t *tripleDocument_;
        bson_iter_t tripleIter_;
    };

} // knowrob

#endif //KNOWROB_MONGO_TRIPLE_CURSOR_H
