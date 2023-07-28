/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/AllocatedQuery.h>

using namespace knowrob;


AllocatedQuery::AllocatedQuery(const std::shared_ptr<const Query> &query,
                               const std::shared_ptr<AnswerStream::Channel> &outputChannel)
: query_(query),
  outputChannel_(outputChannel)
{
}

void AllocatedQuery::pushSolution(const std::shared_ptr<Answer> &solution)
{
	outputChannel_->push(solution);
}

void AllocatedQuery::pushEOS()
{
	outputChannel_->push(AnswerStream::eos());
}
