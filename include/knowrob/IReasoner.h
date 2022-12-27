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
#include <knowrob/terms.h>
#include <knowrob/queries.h>
#include <knowrob/data_sources.h>

namespace knowrob {
	class ReasonerConfiguration {
	public:
		std::list<std::shared_ptr<DataFile>> dataFiles;
		std::list<std::shared_ptr<FactBase>> factBases;
		std::list<std::shared_ptr<RuleBase>> ruleBases;
	};
	
	/**
	 * The interface for reasoning subsystems.
	 */
	class IReasoner {
	public:
		virtual ~IReasoner()= default;

		/** Initialize this reasoner.
		 * This will only be called once when the reasoner is loaded.
		 */
		virtual bool initialize(const ReasonerConfiguration &cfg) = 0;

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
			const std::shared_ptr<QueryResultStream::Channel> &outputStream,
			const std::shared_ptr<Query> &goal) = 0;

		/**
		 */
		virtual void pushSubstitution(uint32_t queryID,
			const SubstitutionPtr &substitution) = 0;
		
		/**
		 */
		virtual void finishQuery(uint32_t queryID,
			bool isImmediateStopRequested) = 0;
	};
}

#endif //__KNOWROB_IREASONER_H__
