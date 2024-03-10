#include <utility>

#include "knowrob/queries/ModalStage.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/modalities/PastModality.h"

using namespace knowrob;

ModalStage::ModalStage(
		KnowledgeBase *kb,
		const std::shared_ptr<ModalFormula> &modal,
		const QueryContextPtr &ctx)
		: QueryStageFormula(modal->modalFormula(), ctx),
		  kb_(kb),
		  modalFormula_(modal) {
	// this modal stage could be embedded into another modal expression.
	// however, forming a mental model of what a human might think about what the robot thinks about him is an
	// additional level which we do not consider (yet).
	// However, expressions like `P(Bp & Bq)` with iterations over different modality types can be used in queries.

	auto modalContext = std::make_shared<QueryContext>(
			*ctx, modalFormula_->modalOperator());
	nestedContext_ = modalContext;

	auto &modalOperator = modalFormula_->modalOperator();

	// modify the knowledge graph selector for the evaluation of the modal formula.
	switch (modalOperator->modality().modalityType()) {
		case ModalityType::Epistemic: {
			// epistemic states of different agents are stored in a single KG where triples are labeled by the agent.
			// These states are assumed to be generated from the perspective of a "self", i.e. the agent
			// that runs the knowledge base and its information about the other agents.
			auto *em = (EpistemicModality *) &modalOperator->modality();

			if (ctx->selector.agent.has_value() && ctx->selector.agent.value() != Perspective::getEgoPerspective()) {
				// For now higher-level epistemic states are not allowed in queries,
				// e.g. `B_a1(B_a2(x))` where `a1` is not the agent that runs the knowledge base is an example
				// of such a higher-order query which is not allowed.
				// `B_self(B_a2(x))` is ok though, and changes the agent label of the KG selector to `a2` below.
				throw QueryError("epistemic formula {} is embedded within epistemic context of another agent.",
								 *modalFormula_);
			}

			modalContext->selector.epistemicOperator = modalOperator->isModalNecessity() ?
													   EpistemicOperator::KNOWLEDGE :
													   EpistemicOperator::BELIEF;
			// TODO: handle confidence parameter of modality operator
			modalContext->selector.confidence = std::nullopt;
			if (em->agent().has_value()) {
				modalContext->selector.agent = em->agent();
			} else {
				modalContext->selector.agent = std::nullopt;
			}
			break;
		}

		case ModalityType::Temporal_Past: {
			auto *pm = (PastModality *) &modalOperator->modality();

			modalContext->selector.temporalOperator = modalOperator->isModalNecessity() ?
													  TemporalOperator::ALWAYS :
													  TemporalOperator::SOMETIMES;

			// TODO: any special treatment for time interval in nested context?
			//		e.g. P(x & P(y)) could only be restricted in evaluation below not here.
			auto &ti = pm->timeInterval();
			if (ti.has_value()) {
				auto &since = ti->since();
				auto &until = ti->until();
				if (since.has_value()) modalContext->selector.begin = since.value().value();
				else modalContext->selector.begin = std::nullopt;
				if (until.has_value()) modalContext->selector.end = until.value().value();
				else modalContext->selector.end = std::nullopt;
			} else {
				modalContext->selector.begin = std::nullopt;
				modalContext->selector.end = std::nullopt;
			}
			break;
		}
	}
}

TokenBufferPtr ModalStage::submitQuery_K(const FormulaPtr &phi) {
	// This case corresponds to "I know that `phi` is true".
	// It is assumed that all the knowledge of an agent is stored in an EDB or can be derived
	// from EDB-facts through top-down methods.
	// Hence, `phi` can be evaluated with the constraint that all positive literals can be grounded
	// in or derived from EDB-facts that are marked as "certain".
	return kb_->submitQuery(phi, nestedContext_);
}

TokenBufferPtr ModalStage::submitQuery_B(const FormulaPtr &phi) {
	// This case corresponds to "I believe that `phi` is true".
	// This is interpreted as a "best guess", and it is assumed that there is only one
	// best guess at a time which is materialized in the EDB.
	// Hence, `phi` can be evaluated with the constraint that all positive literals can be grounded
	// in or derived from EDB-facts that are marked as "certain" or "uncertain":
	return kb_->submitQuery(phi, nestedContext_);
}

TokenBufferPtr ModalStage::submitQuery_P(const FormulaPtr &phi) {
	// This case corresponds to "At some point in the past `phi` was true".
	// It is assumed here that the past is materialized in the EDB already.
	// One use case being the NEEMs of the EASE project that represent episodes
	// of robot actions that happened in the past.
	// TODO: At a later point it might be interesting to consider this case and below as an
	//       access to long term memory where the KG is not materialized locally.
	//       e.g. using the NEEMHub infrastructure of EASE directly for query evaluation.
	//       However, that would have impact on top-down methods that might need to have data locally
	//       if additional reasoning is to be performed.
	// Hence, `phi` can be evaluated with the constraint that all positive literals can be grounded
	// in or derived from EDB-facts whose temporal intersection is non-empty:
	return kb_->submitQuery(phi, nestedContext_);
}

TokenBufferPtr ModalStage::submitQuery_H(const FormulaPtr &phi) {
	// This case corresponds to "At any time in the past `phi` was true".
	// Hence, `phi` can be evaluated with the constraint that all positive literals can be grounded
	// in or derived from EDB-facts that are true "from the beginning of time" until at least the
	// current point in time:
	return kb_->submitQuery(phi, nestedContext_);
}

TokenBufferPtr ModalStage::submitQuery(const FormulaPtr &phi) {
	// FIXME: need to consider more evaluation context? vars in phi are grounded already here in evaluation context. but
	//      - phi could be part of another modal which should be handled here, maybe by throwing an error for now?
	//      - some cases could also be handled in this code, like `KKp <-> Kp` etc. but this is a bit critical...

	// FIXME: the frame of answers that go out this stage need to be manipulated

	switch (modalFormula_->modalOperator()->modality().modalityType()) {
		case ModalityType::Epistemic:
			if (modalFormula_->modalOperator()->isModalNecessity()) {
				return submitQuery_K(phi);
			} else {
				return submitQuery_B(phi);
			}

		case ModalityType::Temporal_Past:
			if (modalFormula_->modalOperator()->isModalNecessity()) {
				return submitQuery_P(phi);
			} else {
				return submitQuery_H(phi);
			}
	}

	throw QueryError("unexpected modality type in query '{}'.", *modalFormula_);
}
