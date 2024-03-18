/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_FACTORY_H_
#define KNOWROB_REASONER_FACTORY_H_

#include <string_view>
#include <memory>
#include "knowrob/reasoner/Reasoner.h"

namespace knowrob {
	/**
	 * Abstract reasoner factory.
	 * Provides an interface for the creation of Reasoner objects.
	 */
	class ReasonerFactory {
	public:
		virtual ~ReasonerFactory() = default;

		/**
		 * Create a new reasoner instance.
		 * @param reasonerID the ID of the reasoner in the knowledge base.
		 * @return the reasoner created.
		 */
		virtual std::shared_ptr<Reasoner> createReasoner(std::string_view reasonerID) = 0;

		/**
		 * @return name of the reasoner type for which the factory can create instances.
		 */
		virtual std::string_view name() const = 0;
	};
}

#endif //KNOWROB_REASONER_FACTORY_H_
