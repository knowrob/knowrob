/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GOAL_DRIVEN_REASONER_H
#define KNOWROB_GOAL_DRIVEN_REASONER_H

#include "Reasoner.h"
#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/queries/TokenBuffer.h"

namespace knowrob {
	/**
	 * A reasoner that supports goal-driven reasoning.
	 * Goal-driven reasoning is a form of reasoning where the reasoner is asked to evaluate a query
	 * and return the results of the query.
	 * This is in contrast to data-driven reasoning, where the reasoner is started and then infers
	 * additional knowledge from the data.
	 */
	class GoalDrivenReasoner : public Reasoner {
	public:
		/**
		 * Get the description of a predicate currently defined by this reasoner.
		 * A predicate is thought to be currently defined if the reasoner can submitQuery it.
		 *
		 * @param indicator a predicate indicator
		 * @return a predicate description if the predicate is a defined one or null otherwise.
		 */
		virtual PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) = 0;

		/**
		 * Get the description of the predicate which is associated with a literal.
		 * A null reference will be returned in case that the property term of the literal is a variable.
		 * TODO: revise this interface, maybe better that reasoner explicitly register defined predicates.
		 * @param literal a literal.
		 * @return a predicate description or a null reference.
		 */
		PredicateDescriptionPtr getLiteralDescription(const FramedTriplePattern &literal);

		/**
		 * Submit a query to the reasoner.
		 * The query is represented by a literal and a context.
		 * The evaluation of the query is performed asynchronously, the result of this function
		 * is a buffer that can be used to retrieve the results of the query at a later point in time.
		 * @param literal a literal representing the query.
		 * @param ctx a query context.
		 * @return a buffer that can be used to retrieve the results of the query.
		 */
		virtual TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) = 0;
	};

	using GoalDrivenReasonerPtr = std::shared_ptr<GoalDrivenReasoner>;
}

#endif //KNOWROB_GOAL_DRIVEN_REASONER_H
