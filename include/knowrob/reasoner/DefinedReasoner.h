/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEFINED_REASONER_H_
#define KNOWROB_DEFINED_REASONER_H_

#include <string>
#include <memory>
#include "knowrob/reasoner/Reasoner.h"

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
		DefinedReasoner(std::string_view name, const std::shared_ptr<Reasoner> &reasoner)
		: name_(name), reasoner_(reasoner) {}

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<Reasoner>& operator()() const { return reasoner_; }

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<Reasoner>& reasoner() const { return reasoner_; }

		/**
		 * @return the reasoner name.
		 */
		const std::string& name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<Reasoner> reasoner_;
	};
}

#endif //KNOWROB_DEFINED_REASONER_H_
