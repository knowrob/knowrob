/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BACKEND_FACTORY_H_
#define KNOWROB_BACKEND_FACTORY_H_

#include <string>
#include <memory>
#include "DefinedBackend.h"

namespace knowrob {
	/**
	 * Abstract backend factory.
	 * Provides an interface for the creation of DataBackend instances.
	 */
	class BackendFactory {
	public:
		virtual ~BackendFactory()= default;

		/**
		 * Create a new DataBackend instance.
		 * @param backendID the ID of the backend in the knowledge base.
		 * @return the reasoner created.
		 */
		virtual std::shared_ptr<DefinedBackend> createBackend(const std::string &backendID) = 0;

		/**
		 * @return name of the KG type for which the factory can create instances.
		 */
		virtual const std::string& name() const = 0;
	};
}

#endif //KNOWROB_BACKEND_FACTORY_H_
