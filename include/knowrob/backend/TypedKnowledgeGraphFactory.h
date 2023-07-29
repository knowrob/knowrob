/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TYPED_KG_FACTORY_H_
#define KNOWROB_TYPED_KG_FACTORY_H_

#include <string>
#include <memory>
#include "KnowledgeGraphFactory.h"
#include "KnowledgeGraph.h"

namespace knowrob {
	/**
	 * A KG factory implementation for builtin types.
	 * @tparam T the type of KG.
	 */
	template<class T> class TypedKnowledgeGraphFactory : public KnowledgeGraphFactory {
	public:
		/**
		 * @param name name of the KG type for which the factory can create instances.
		 */
		explicit TypedKnowledgeGraphFactory(std::string name) : name_(std::move(name)) {}

		// Override BackendFactory
		std::shared_ptr<DefinedKnowledgeGraph> createKnowledgeGraph(const std::string &backendID) override
		{ return std::make_shared<DefinedKnowledgeGraph>(backendID, std::make_shared<T>()); }

		// Override BackendFactory
		const std::string& name() const override {  return name_; }
	protected:
		std::string name_;
	};
}

#endif //KNOWROB_TYPED_KG_FACTORY_H_
