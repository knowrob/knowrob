/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GOAL_DRIVEN_REASONER_H
#define KNOWROB_GOAL_DRIVEN_REASONER_H

#include "Reasoner.h"
#include "knowrob/queries/TokenBuffer.h"
#include "knowrob/formulas/PredicateIndicator.h"

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
		GoalDrivenReasoner() : Reasoner() {}

		/**
		 * Find out if the relation is defined by this reasoner.
		 * A defined relation is a relation that is known to the reasoner, and
		 * for which the reasoner can provide additional groundings when being queried.
		 * @param indicator a predicate indicator.
		 * @return true if the relation is currently defined by this reasoner.
		 */
		bool isRelationDefined(const PredicateIndicator &indicator) {
			return definedRelations_.find(indicator) != definedRelations_.end();
		}

		/**
		 * Add a defined relation to the reasoner.
		 * @param indicator a predicate indicator.
		 */
		void defineRelation(const PredicateIndicator &indicator) { definedRelations_.emplace(indicator); }

		/**
		 * Remove a defined relation from the reasoner.
		 * @param indicator a predicate indicator.
		 */
		void unDefineRelation(const PredicateIndicator &indicator) { definedRelations_.erase(indicator); }

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

	protected:
		std::set<PredicateIndicator> definedRelations_;
	};

	using GoalDrivenReasonerPtr = std::shared_ptr<GoalDrivenReasoner>;
}

#endif //KNOWROB_GOAL_DRIVEN_REASONER_H
