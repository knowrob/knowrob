/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DEFINED_KG_H_
#define KNOWROB_DEFINED_KG_H_

#include <string>
#include <memory>
#include "KnowledgeGraph.h"

namespace knowrob {
	/**
	 * A KG with a name managed by the KG manager.
	 */
	class DefinedKnowledgeGraph {
	public:
		/**
		 * @param name the name of the KG, unique within manager
		 * @param backend the KG instance
		 */
		DefinedKnowledgeGraph(std::string name, const std::shared_ptr<KnowledgeGraph> &backend)
		: name_(std::move(name)), knowledgeGraph_(backend) {}

		/**
		 * @return the KG instance
		 */
		const std::shared_ptr<KnowledgeGraph>& operator()() const { return knowledgeGraph_; }

		/**
		 * @return the KG instance
		 */
		const std::shared_ptr<KnowledgeGraph>& knowledgeGraph() const { return knowledgeGraph_; }

		/**
		 * @return the KG name.
		 */
		const std::string& name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<KnowledgeGraph> knowledgeGraph_;
	};
}

#endif //KNOWROB_DEFINED_KG_H_
