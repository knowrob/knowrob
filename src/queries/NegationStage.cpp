//
// Created by danielb on 23.11.23.
//

#include <utility>

#include "knowrob/queries/NegationStage.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/formulas/ModalFormula.h"

using namespace knowrob;

NegationStage::NegationStage(KnowledgeBase *kb, QueryContextPtr ctx)
: AnswerBroadcaster(),
  kb_(kb),
  ctx_(std::move(ctx))
{}

void NegationStage::pushToBroadcast(const AnswerPtr &msg) {
	if (isEOS(msg) || succeeds(msg)) {
		AnswerBroadcaster::pushToBroadcast(msg);
	}
}

LiteralNegationStage::LiteralNegationStage(KnowledgeBase *kb,
							               const QueryContextPtr &ctx,
							               const std::vector<RDFLiteralPtr> &negatedLiterals)
: NegationStage(kb,ctx),
  negatedLiterals_(negatedLiterals)
{}

bool LiteralNegationStage::succeeds(const AnswerPtr &answer)
{
	// here the idea is to issue individual queries for each negative literal.
	// each query produces an AnswerBufferPtr object.
	// the set of all results can then be checked for a certain property, e.g. that none or all
	// of the queries produced a result.
	std::vector<AnswerBufferPtr> results;

    std::shared_ptr<KnowledgeGraph> kg = kb_->centralKG();

	for (auto &lit: negatedLiterals_) {
		auto lit1 = lit->applySubstitution(*answer->substitution());
		// for now evaluate positive variant of the literal.
		// NOTE: for open-world semantics this cannot be done. open-world reasoner
		//       would need to receive negative literal instead.
		lit1->setIsNegated(false);

		// create an instance of the literal based on given substitution
		auto instance = RDFLiteral::fromLiteral(lit1, ctx_->selector_);

		// check if the EDB contains positive lit, if so negation cannot be true
		results.push_back(kg->submitQuery(
				std::make_shared<GraphQuery>(instance, ctx_)));

		// next check if positive lit is an IDB predicate, if so negation cannot be true.
		// get list of reasoner that define the literal
		// TODO: could be done faster through interface in manager
		std::vector<std::shared_ptr<Reasoner>> l_reasoner;
		for (auto &pair: kb_->reasonerManager()->reasonerPool()) {
			auto &r = pair.second->reasoner();
			if (r->canEvaluate(*instance)) {
				results.push_back(r->submitQuery(instance, ctx_));
			}
		}
	}

	// go through results, ensure *all* sub-queries failed, only then the negation stage succeeds.
	for (auto &result: results) {
		auto resultQueue = result->createQueue();
		auto firstResult = resultQueue->pop_front();
		if (!isEOS(firstResult)) return false;
	}

	return true;
}

ModalNegationStage::ModalNegationStage(KnowledgeBase *kb,
                                       const QueryContextPtr &ctx,
                                       const std::vector<std::shared_ptr<ModalFormula>> &negatedModals)
: NegationStage(kb,ctx),
  negatedModals_(negatedModals)
{}

bool ModalNegationStage::succeeds(const AnswerPtr &answer)
{
	// here the idea is to issue individual queries for each negated modal.
	// for now the positive variants of the modals is evaluated, the modal negation is thought to be
	// true if none of the positive variants can be grounded.

	auto outputStream = std::make_shared<AnswerBuffer>();

	// run "submitQuery" for each negated modal, and collect results in outputStream
	for (auto &modal: negatedModals_) {
		auto modalInstance = modal->applySubstitution(*answer->substitution());
		auto modalOutput = kb_->submitQuery(modalInstance, ctx_);
		modalOutput >> outputStream;
		modalOutput->stopBuffering();
	}

	// wait for and pop first message from output stream
	auto resultQueue = outputStream->createQueue();
	auto firstResult = resultQueue->pop_front();
	// "EOS" indicates that no solution was found for any of the modals
	return isEOS(firstResult);
}
