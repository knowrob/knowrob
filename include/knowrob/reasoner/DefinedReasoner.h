/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEFINED_REASONER_H_
#define KNOWROB_DEFINED_REASONER_H_

#include <string>
#include <memory>
#include "knowrob/reasoner/IReasoner.h"

namespace knowrob {
	/**
	 * A reasoner with a name managed by the reasoner manager.
	 */
	class DefinedReasoner {
	public:
		/**
		 * @param name the name of the reasoner, unique within manager
		 * @param reasoner the reasoner instance
		 */
		DefinedReasoner(std::string name, const std::shared_ptr<IReasoner> &reasoner)
		: name_(std::move(name)), reasoner_(reasoner) {}

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<IReasoner>& operator()() const { return reasoner_; }

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<IReasoner>& reasoner() const { return reasoner_; }

		/**
		 * @return the reasoner name.
		 */
		const std::string& name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<IReasoner> reasoner_;
	};
}

#endif //KNOWROB_DEFINED_REASONER_H_
