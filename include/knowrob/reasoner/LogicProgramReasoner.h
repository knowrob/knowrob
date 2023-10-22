/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_LOGIC_PROGRAM_REASONER_H_
#define KNOWROB_LOGIC_PROGRAM_REASONER_H_

// STD
#include <list>
#include <memory>
// KnowRob
#include "Reasoner.h"

namespace knowrob {
	/**
	 */
	class LogicProgramReasoner : public Reasoner {
	public:
		LogicProgramReasoner() = default;

		/** Asserts a fact in the knowledge base.
		 * @predicate a grounded predicate.
		 */
		virtual bool assertFact(const std::shared_ptr<Predicate> &predicate) = 0;

		/** Get the fact bases of the logic program.
		 *
		 * @return the fact bases
		 */
		//const std::list<std::shared_ptr<IFactBase>>& edbs() const { return edbs_; }

		/** Get the fact bases of the logic program.
		 *
		 * @return the fact bases
		 */
		//const std::list<std::shared_ptr<IRuleBase>>& idbs() const { return idbs_; }

		//void addEDB(std::shared_ptr<IFactBase> &edb) { edbs_.push_back(edb); }
		//void addIDB(std::shared_ptr<IRuleBase> &idb) { idbs_.push_back(idb); }

	protected:
		//std::list<std::shared_ptr<IFactBase>> edbs_;
		//std::list<std::shared_ptr<IRuleBase>> idbs_;
	};
}

#endif //KNOWROB_LOGIC_PROGRAM_REASONER_H_
