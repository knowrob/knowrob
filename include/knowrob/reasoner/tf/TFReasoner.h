/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TF_REASONER_H
#define KNOWROB_TF_REASONER_H

#include "knowrob/reasoner/ComputableReasoner.h"

namespace knowrob {

	class TFReasoner : public ComputableReasoner {
	public:
        /**
         * @param reasonerID a knowledge base identifier of this reasoner.
         */
        explicit TFReasoner(std::string reasonerID);

        ~TFReasoner() override;

        // Override IReasoner
        void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) override;

        // Override IReasoner
        unsigned long getCapabilities() const override;

		// Override IReasoner
		bool loadConfiguration(const ReasonerConfiguration &cfg) override;

		static void tfGetPose(const std::shared_ptr<knowrob::RDFLiteral> &literal,
					   const std::shared_ptr<AnswerStream::Channel> &outputChannel);


    protected:
		void tfMemClear(const std::shared_ptr<knowrob::RDFLiteral> &literal,
						const std::shared_ptr<AnswerStream::Channel> &outputChannel);
	};

} // knowrob

#endif //KNOWROB_TF_REASONER_H
