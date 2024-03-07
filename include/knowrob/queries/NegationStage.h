/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_NEGATION_STAGE_H
#define KNOWROB_NEGATION_STAGE_H

#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/formulas/ModalFormula.h"
#include "AnswerYes.h"

namespace knowrob {
	/**
	 * A stage that evaluates a set of negations which are considered
	 * in conjunction.
	 */
	class NegationStage : public TokenBroadcaster {
	public:
		NegationStage(KnowledgeBase *kb, QueryContextPtr ctx);

	protected:
		KnowledgeBase *kb_;
		const QueryContextPtr ctx_;

		void pushToBroadcast(const TokenPtr &tok) override;

		virtual bool succeeds(const AnswerYesPtr &answer) = 0;
	};

	class LiteralNegationStage : public NegationStage {
	public:
		LiteralNegationStage(KnowledgeBase *kb,
							 const QueryContextPtr &ctx,
							 const std::vector<FramedTriplePatternPtr> &negatedLiterals);

	protected:
		const std::vector<FramedTriplePatternPtr> negatedLiterals_;

		bool succeeds(const AnswerYesPtr &answer) override;
	};

	class ModalNegationStage : public NegationStage {
	public:
		ModalNegationStage(KnowledgeBase *kb,
						   const QueryContextPtr &ctx,
						   const std::vector<std::shared_ptr<ModalFormula>> &negatedModals);

	protected:
		const std::vector<std::shared_ptr<ModalFormula>> negatedModals_;

		bool succeeds(const AnswerYesPtr &answer) override;
	};

} // knowrob

#endif //KNOWROB_NEGATION_STAGE_H
