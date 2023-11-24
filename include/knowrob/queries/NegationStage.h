//
// Created by danielb on 23.11.23.
//

#ifndef KNOWROB_NEGATION_STAGE_H
#define KNOWROB_NEGATION_STAGE_H

#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/reasoner/ReasonerManager.h"

namespace knowrob {
	/**
	 * A stage that evaluates a set of negations which are considered
	 * in conjunction.
	 */
	class NegationStage : public AnswerBroadcaster {
	public:
		NegationStage(const std::shared_ptr<KnowledgeGraph> &kg,
					  const std::shared_ptr<ReasonerManager> &reasonerManager,
					  const std::vector<RDFLiteralPtr> &negativeLiterals);

		bool succeeds(const AnswerPtr &answer);

	protected:
		std::shared_ptr<KnowledgeGraph> kg_;
		std::shared_ptr<ReasonerManager> reasonerManager_;
		const std::vector<RDFLiteralPtr> negativeLiterals_;

		void pushToBroadcast(const AnswerPtr &msg) override;
	};

} // knowrob

#endif //KNOWROB_NEGATION_STAGE_H
