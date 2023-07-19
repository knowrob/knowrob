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
#include "KnowledgeGraph.h"

namespace knowrob {
	/**
	 * Abstract knowledgeGraph factory.
	 * Provides an interface for the creation of KnowledgeGraph instances.
	 */
	class KnowledgeGraphFactory {
	public:
		virtual ~KnowledgeGraphFactory()= default;

		/**
		 * Create a new KnowledgeGraph instance.
		 * @param backendID the ID of the knowledgeGraph in the knowledge base.
		 * @return the reasoner created.
		 */
		virtual std::shared_ptr<KnowledgeGraph> createKnowledgeGraph(const std::string &backendID) = 0;

		/**
		 * @return name of the knowledgeGraph type for which the factory can create instances.
		 */
		virtual const std::string& name() const = 0;
	};
}

#endif //KNOWROB_BACKEND_FACTORY_H_
