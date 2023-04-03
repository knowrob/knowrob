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
	class QueryInstance {
	public:
		/**
		 * @param uninstantiatedQuery a query that may contain variables
		 * @param outputChannel an output channel where solutions are pushed
		 * @param partialResult a partial result from sub-queries evaluated before
		 */
		QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
					  const std::shared_ptr<AnswerStream::Channel> &outputChannel,
					  const std::shared_ptr<const Answer> &partialResult);
		/**
		 * @param uninstantiatedQuery a query that may contain variables
		 * @param outputChannel an output channel where solutions are pushed
		 */
		QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
					  const std::shared_ptr<AnswerStream::Channel> &outputChannel);

		/**
		 * @return an instane of the input query.
		 */
		std::shared_ptr<const Query> create();

		/**
		 * Push a new solution for the instantiated query into the QA pipeline.
		 * @param solution a query solution
		 */
		void pushSolution(const std::shared_ptr<Answer> &solution);

		/**
		 * Push EOS message indicating that no more solutions will be generated.
		 */
		void pushEOS();

		/**
		 * @return the uninstantiated query.
		 */
		const std::shared_ptr<const Query>& uninstantiatedQuery() const { return uninstantiatedQuery_; }

		/**
		 * @return an optional time interval of this query.
		 */
		const std::optional<const TimeInterval*>& timeInterval() const;

		/**
		 * @return an optional confidenceInterval interval of this query.
		 */
		const std::optional<const ConfidenceInterval*>& confidenceInterval() const;

	protected:
		std::shared_ptr<const Query> uninstantiatedQuery_;
		std::shared_ptr<const Answer> partialResult_;
		std::shared_ptr<AnswerStream::Channel> outputChannel_;
	};
	using QueryInstancePtr = std::shared_ptr<QueryInstance>;
}

#endif //KNOWROB_QUERY_INSTANCE_H_
