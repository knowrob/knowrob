/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGOLOG_REASONER_H_
#define KNOWROB_MONGOLOG_REASONER_H_

// STD
#include <memory>
// KnowRob
#include "knowrob/terms/Term.h"
#include "knowrob/backend/mongo/MongoKnowledgeGraph.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"

namespace knowrob {
	class MongologReasoner : public PrologReasoner {
	public:
		MongologReasoner();

		~MongologReasoner() override;

		//bool projectIntoEDB(const Statement &statement) override;

		//bool exportData(const std::filesystem::path &path) override;

		//bool importData(const std::filesystem::path &path) override;

		bool initializeReasoner(const PropertyTree &cfg) override;

		void setDataBackend(const DataBackendPtr &backend) override;

		const auto &knowledgeGraph() const { return knowledgeGraph_; }

	protected:
		std::shared_ptr<MongoKnowledgeGraph> knowledgeGraph_;

		// Override PrologReasoner
		std::string_view callFunctor() override;

		// Override PrologReasoner
		bool initializeDefaultPackages() override;
	};
}

#endif //KNOWROB_MONGOLOG_REASONER_H_
