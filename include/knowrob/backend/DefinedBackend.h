/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEFINED_BACKEND_H_
#define KNOWROB_DEFINED_BACKEND_H_

#include <string>
#include <memory>
#include "DataBackend.h"

namespace knowrob {
	/**
	 * A backend with a name managed by the backend manager.
	 */
	class DefinedBackend {
	public:
		/**
		 * @param name the name of the backend, unique within manager
		 * @param backend the KG instance
		 */
		DefinedBackend(std::string name, const std::shared_ptr<DataBackend> &backend)
		: name_(std::move(name)), backend_(backend) {}

		/**
		 * @return the backend instance
		 */
		auto& operator()() const { return backend_; }

		/**
		 * @return the backend instance
		 */
		auto& backend() const { return backend_; }

		/**
		 * @return the backend name.
		 */
		auto& name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<DataBackend> backend_;
	};
}

#endif //KNOWROB_DEFINED_BACKEND_H_
