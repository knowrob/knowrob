/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

// STD
#include <string>
#include <memory>
// KnowRob
#include <knowrob/lang/terms.h>
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/LogicProgramReasoner.h>
#include <knowrob/reasoning/prolog/PrologPool.h>

namespace knowrob {
	/**
	 * A Prolog reasoner that performs reasoning using SWI Prolog.
	 */
	class PrologReasoner : public LogicProgramReasoner {
	public:
		PrologReasoner(
			std::shared_ptr<PrologPool> &prologEnginePool,
			const std::string &initFile);
		~PrologReasoner();

		/**
		 * Consults a Prolog file, i.e. loads facts and rules and executed
		 * directives in the file.
		 * May throw an exception if there is no valid Prolog file at the given path.
		 */
		void consult(const std::string &prologFile);

		void assert(const std::shared_ptr<Predicate> &predicate);

		// Override IReasoner::initialize
 		void initialize();

		// Override IReasoner::runQuery
		void runQuery(
			std::shared_ptr<Query> &goal,
			ReasoningStatus &status,
			std::shared_ptr<QueryResultQueue> &answerQueue);

		// Override IReasoner::canReasonAbout
		bool canReasonAbout(const PredicateIndicator &predicate);

	protected:
		std::shared_ptr<PrologPool> prologEnginePool_;
		std::string initFile_;
	};
}

#endif //__KNOWROB_PROLOG_REASONER_H__
