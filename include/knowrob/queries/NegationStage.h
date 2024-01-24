//
// Created by danielb on 23.11.23.
//

#ifndef KNOWROB_NEGATION_STAGE_H
#define KNOWROB_NEGATION_STAGE_H

#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/formulas/ModalFormula.h"

namespace knowrob {
	/**
	 * A stage that evaluates a set of negations which are considered
	 * in conjunction.
	 */
	class NegationStage : public AnswerBroadcaster {
	public:
		NegationStage(KnowledgeBase *kb, QueryContextPtr ctx);

	protected:
		KnowledgeBase *kb_;
		const QueryContextPtr ctx_;

		void pushToBroadcast(const AnswerPtr &msg) override;
		virtual bool succeeds(const AnswerPtr &answer) = 0;
	};

	class LiteralNegationStage : public NegationStage {
	public:
		LiteralNegationStage(KnowledgeBase *kb,
		                     const QueryContextPtr &ctx,
		                     const std::vector<RDFLiteralPtr> &negatedLiterals);

	protected:
		const std::vector<RDFLiteralPtr> negatedLiterals_;
		bool succeeds(const AnswerPtr &answer) override;
	};

	class ModalNegationStage : public NegationStage {
	public:
		ModalNegationStage(KnowledgeBase *kb,
						   const QueryContextPtr &ctx,
						   const std::vector<std::shared_ptr<ModalFormula>> &negatedModals);

	protected:
		const std::vector<std::shared_ptr<ModalFormula>> negatedModals_;
		bool succeeds(const AnswerPtr &answer) override;
	};

} // knowrob

#endif //KNOWROB_NEGATION_STAGE_H
