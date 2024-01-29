/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TYPED_BACKEND_FACTORY_H_
#define KNOWROB_TYPED_BACKEND_FACTORY_H_

#include <string>
#include <memory>
#include "knowrob/backend/BackendFactory.h"
#include "DefinedBackend.h"

namespace knowrob {
	/**
	 * A backend factory implementation for builtin types.
	 * @tparam T the type of KG.
	 */
	template<class T> class TypedBackendFactory : public BackendFactory {
	public:
		/**
		 * @param name name of the backend type for which the factory can create instances.
		 */
		explicit TypedBackendFactory(std::string name) : name_(std::move(name)) {}

		// Override BackendFactory
		std::shared_ptr<DefinedBackend> createBackend(const std::string &backendID) override
		{ return std::make_shared<DefinedBackend>(backendID, std::make_shared<T>()); }

		// Override BackendFactory
		const std::string& name() const override {  return name_; }
	protected:
		std::string name_;
	};
}

#endif //KNOWROB_TYPED_BACKEND_FACTORY_H_
