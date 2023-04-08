//
// Created by daniel on 08.04.23.
//

#ifndef KNOWROB_MONGO_ANSWER_CURSOR_H
#define KNOWROB_MONGO_ANSWER_CURSOR_H

#include "Cursor.h"
#include "knowrob/queries/Answer.h"

namespace knowrob::mongo {
    class AnswerCursor : public Cursor {
    public:
        explicit AnswerCursor(const std::shared_ptr<Collection> &collection);

        bool nextAnswer(const std::shared_ptr<Answer> &answer);

    protected:
        const bson_t *resultDocument_;
        bson_iter_t resultIter_;
        bson_iter_t varIter_;
        bson_iter_t valIter_;
    };
} // mongo

#endif //KNOWROB_ANSWERCURSOR_H
