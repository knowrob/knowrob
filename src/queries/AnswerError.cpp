/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <sstream>
#include <knowrob/queries/AnswerError.h>

using namespace knowrob;

AnswerError::AnswerError(
        const Answer &erroneousAnswer,
        const Term &errorTerm)
        : std::runtime_error(formatErrorString(erroneousAnswer, errorTerm))
{
}

std::string AnswerError::formatErrorString(
        const Answer &erroneousAnswer,
        const Term &errorTerm)
{
    std::stringstream buffer;
    buffer << errorTerm;
    return buffer.str();
}
