/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ANSWER_ERROR_H_
#define KNOWROB_ANSWER_ERROR_H_

#include <string>
#include <fmt/core.h>
#include "knowrob/terms/Term.h"
#include "Answer.h"
#include <knowrob/queries/ModalQuery.h>

namespace knowrob {
    /**
     * A answer-related runtime error.
     */
    class AnswerError : public std::runtime_error {
    public:
        /**
         * @tparam Args fmt-printable arguments.
         * @param fmt A fmt string pattern.
         * @param args list of arguments used to instantiate the pattern.
         */
        template<typename ... Args> explicit AnswerError(const char *fmt, Args&& ... args)
                : std::runtime_error(fmt::format(fmt, args...))
        {}

        /**
         * @param erroneousQuery the query that caused an error
         * @param errorTerm a term denoting the error
         */
        AnswerError(const Answer &erroneousAnswer, const Term &errorTerm);

    protected:

        static std::string formatErrorString(const Answer &erroneousAnswer, const Term &errorTerm);
    };
}

#endif //KNOWROB_ANSWER_ERROR_H_
