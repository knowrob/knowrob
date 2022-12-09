/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_IREASONER_H__
#define __KNOWROB_IREASONER_H__

// STD
#include <memory>
// KnowRob
#include <knowrob/lang/terms.h>
#include <knowrob/qa/queries.h>

namespace knowrob {
	/**
	 * The interface for reasoning subsystems.
	 */
	class IReasoner {
	public:
		virtual ~IReasoner(){}

		/** Initialize this reasoner.
		 * This will only be called once when the reasoner is loaded.
		 * @todo is it certain run is not called while initialization is still ongoing?
		 */
		virtual void initialize() = 0;

		/** Find out whether this reasoner can handle a given predicate.
		 *
		 * Note that, following the Syntax of the querying language,
		 * for a reasoner to be able to answer the `,\2` predicate entails that the
		 * reasoner can handle conjunctive queries, `;\2` disjunctive queries etc.
		 *
		 * @param predicate the predicate in question
		 * @return true if the reasoner can determine the truth of given predicate.
		 */
		virtual bool canReasonAbout(const PredicateIndicator &predicate) = 0;

		/**
		 */
		virtual void startQuery(uint32_t queryID,
			const std::shared_ptr<QueryResultStream> &outputStream,
			const std::shared_ptr<Query> &goal) = 0;

		/**
		 */
		virtual void pushQueryBindings(uint32_t queryID,
			const QueryResultPtr &bindings) = 0;
		
		/**
		 */
		virtual void finishQuery(uint32_t queryID,
			bool isImmediateStopRequested) = 0;
	};
}

#endif //__KNOWROB_IREASONER_H__
