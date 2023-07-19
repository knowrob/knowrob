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
#include "KnowledgeGraph.h"

namespace knowrob {
	/**
	 * A reasoner with a name managed by the reasoner manager.
	 */
	class DefinedKnowledgeGraph {
	public:
		/**
		 * @param name the name of the reasoner, unique within manager
		 * @param reasoner the reasoner instance
		 */
		DefinedKnowledgeGraph(std::string name, const std::shared_ptr<KnowledgeGraph> &backend)
		: name_(std::move(name)), knowledgeGraph_(backend) {}

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<KnowledgeGraph>& operator()() const { return knowledgeGraph_; }

		/**
		 * @return the reasoner instance
		 */
		const std::shared_ptr<KnowledgeGraph>& knowledgeGraph() const { return knowledgeGraph_; }

		/**
		 * @return the reasoner name.
		 */
		const std::string& name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<KnowledgeGraph> knowledgeGraph_;
	};
}

#endif //KNOWROB_DEFINED_REASONER_H_
