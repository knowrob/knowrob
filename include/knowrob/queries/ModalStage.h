//
// Created by daniel on 31.08.23.
//

#ifndef KNOWROB_MODAL_STAGE_H
#define KNOWROB_MODAL_STAGE_H

#include "QueryStage.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/formulas/ModalFormula.h"

namespace knowrob {
	/**
	 * Evaluates a formula under a given modality.
	 */
    class ModalStage : public FormulaQueryStage {
    public:
        ModalStage(KnowledgeBase *kb,
                   const std::shared_ptr<ModalFormula> &modalFormula,
                   const QueryContextPtr &ctx);

    protected:
        KnowledgeBase *kb_;
		std::shared_ptr<ModalFormula> modalFormula_;
		QueryContextPtr nestedContext_;

        AnswerBufferPtr submitQuery(const FormulaPtr &modalInstance) override;

        AnswerBufferPtr submitQuery_K(const FormulaPtr &modalInstance);
        AnswerBufferPtr submitQuery_B(const FormulaPtr &modalInstance);
        AnswerBufferPtr submitQuery_P(const FormulaPtr &modalInstance);
        AnswerBufferPtr submitQuery_H(const FormulaPtr &modalInstance);
    };

} // knowrob

#endif //KNOWROB_MODAL_STAGE_H
