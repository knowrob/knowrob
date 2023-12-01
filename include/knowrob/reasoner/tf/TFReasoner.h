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
        std::shared_ptr<PredicateDescription> getPredicateDescription(
                const std::shared_ptr<PredicateIndicator> &indicator) override;

        // Override IReasoner
        unsigned long getCapabilities() const override;

        // Override IReasoner
        AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal, int queryFlags) override;

        // Override IReasoner
        bool loadDataSourceWithUnknownFormat(const DataSourcePtr &dataFile) override;

		static void tfGetPose(const std::shared_ptr<knowrob::RDFLiteral> &literal,
					   const std::shared_ptr<AnswerStream::Channel> &outputChannel);


    protected:
	};

} // knowrob

#endif //KNOWROB_TF_REASONER_H
