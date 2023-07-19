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
#include "KnowledgeGraphFactory.h"
#include "KnowledgeGraph.h"

namespace knowrob {
	/**
	 * A backend factory implementation for builtin knowledgeGraph types.
	 * @tparam T the type of knowledgeGraph.
	 */
	template<class T> class TypedKnowledgeGraphFactory : public KnowledgeGraphFactory {
	public:
		/**
		 * @param name name of the knowledgeGraph type for which the factory can create instances.
		 */
		explicit TypedKnowledgeGraphFactory(std::string name) : name_(std::move(name)) {}

		// Override BackendFactory
		std::shared_ptr<KnowledgeGraph> createKnowledgeGraph(const std::string &backendID) override
		{ return std::make_shared<T>(backendID); }

		// Override BackendFactory
		const std::string& name() const override {  return name_; }
	protected:
		std::string name_;
	};
}

#endif //KNOWROB_TYPED_BACKEND_FACTORY_H_
