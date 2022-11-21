/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_IANSWER_PUBLISHER_H__
#define __KNOWROB_IANSWER_PUBLISHER_H__

#include <list>
// boost
#include <boost/shared_ptr.hpp>

#include "knowrob/lang/IQuery.h"
#include "knowrob/lang/Answer.h"
#include "knowrob/qa/IAnswerListener.h"

namespace knowrob {
    /**
     * Stores a list of answers generated and broadcasts them to listeners.
     */
    class IAnswerPublisher {
    public:
        IAnswerPublisher(const boost::shared_ptr<IQuery> &query);
        ~IAnswerPublisher();

        /** Add a listener for generated answers.
         *
         * Note that the listener is also invoked for each answer generated
         * before the listener was added.
         *
         * @param answer the answer to publish
         */
        void addListener(const boost::shared_ptr<IAnswerListener> &listener);

        /** Removes a previously added listener.
         *
         * @param answer the answer to publish
         */
        void removeListener(const boost::shared_ptr<IAnswerListener> &listener);

        /** Publish an answer.
         *
         * @param answer the answer to publish
         */
        void pushAnswer(const boost::shared_ptr<Answer> &answer);

        /** Get list of answer listener.
         *
         * @return list of answer listener
         */
        const std::list<boost::shared_ptr<IAnswerListener>>& listener() const { return listener_; }

        /** Get list of answera.
         *
         * @return list of answers
         */
        const std::list<boost::shared_ptr<Answer>>& answers() const { return answers_; }

    private:
        boost::shared_ptr<IQuery> goal_;
        std::list<boost::shared_ptr<Answer>> answers_;

        std::list<boost::shared_ptr<IAnswerListener>> listener_;
    };
}

#endif //__KNOWROB_IANSWER_PUBLISHER_H__