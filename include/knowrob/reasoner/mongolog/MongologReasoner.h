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
#include "knowrob/mongodb/MongoKnowledgeGraph.h"
#include "knowrob/reasoner/IReasoner.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"

namespace knowrob {
	class MongologReasoner : public PrologReasoner {
	public:
		explicit MongologReasoner(const std::string &reasonerID);

		~MongologReasoner() override;

        unsigned long getCapabilities() const override;

        bool projectIntoEDB(const Statement &statement) override;

        bool exportData(const std::filesystem::path &path) override;

        bool importData(const std::filesystem::path &path) override;

        bool loadConfiguration(const ReasonerConfiguration &cfg) override;

        const auto& knowledgeGraph() const { return knowledgeGraph_; }
		
	protected:
	    std::shared_ptr<MongoKnowledgeGraph> knowledgeGraph_;

		// Override PrologReasoner
		const functor_t& callFunctor() override;

		// Override PrologReasoner
		bool initializeDefaultPackages() override;
    };
}

#endif //KNOWROB_MONGOLOG_REASONER_H_
