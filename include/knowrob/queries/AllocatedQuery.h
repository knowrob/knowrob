/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_INSTANCE_H_
#define KNOWROB_QUERY_INSTANCE_H_

#include <memory>
#include <optional>
#include <knowrob/queries/Query.h>
#include <knowrob/queries/Answer.h>
#include <knowrob/queries/AnswerStream.h>

namespace knowrob {
	/**
	 * An instantiation of a query, i.e. where some variables may be substituted by terms.
	 */
	class AllocatedQuery {
	public:
		AllocatedQuery(const std::shared_ptr<const Query> &query,
                       const std::shared_ptr<AnswerStream::Channel> &outputChannel);

        const std::shared_ptr<const Query>& query() const { return query_; }

        auto& formula() const { return query_->formula(); }

        auto& modalFrame() const { return query_->modalFrame(); }

		/**
		 * Push a new solution for the instantiated query into the QA pipeline.
		 * @param solution a query solution
		 */
		void pushSolution(const std::shared_ptr<Answer> &solution);

		/**
		 * Push EOS message indicating that no more solutions will be generated.
		 */
		void pushEOS();

	protected:
		std::shared_ptr<const Query> query_;
		std::shared_ptr<AnswerStream::Channel> outputChannel_;
	};
	using AllocatedQueryPtr = std::shared_ptr<AllocatedQuery>;
}

#endif //KNOWROB_QUERY_INSTANCE_H_
