/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_FACTORY_H_
#define KNOWROB_REASONER_FACTORY_H_

#include <string>
#include <memory>
#include "knowrob/reasoner/IReasoner.h"

namespace knowrob {
	/**
	 * Abstract reasoner factory.
	 * Provides an interface for the creation of IReasoner instances.
	 */
	class ReasonerFactory {
	public:
		virtual ~ReasonerFactory()= default;

		/**
		 * Create a new reasoner instance.
		 * @param reasonerID the ID of the reasoner in the knowledge base.
		 * @return the reasoner created.
		 */
		virtual std::shared_ptr<IReasoner> createReasoner(const std::string &reasonerID) = 0;

		/**
		 * @return name of the reasoner type for which the factory can create instances.
		 */
		virtual const std::string& name() const = 0;
	};
}

#endif //KNOWROB_REASONER_FACTORY_H_
