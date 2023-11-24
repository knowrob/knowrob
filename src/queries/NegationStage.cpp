//
// Created by danielb on 23.11.23.
//

#include "knowrob/queries/NegationStage.h"
#include "knowrob/KnowledgeBase.h"

using namespace knowrob;

NegationStage::NegationStage(const std::shared_ptr<KnowledgeGraph> &kg,
							 const std::shared_ptr<ReasonerManager> &reasonerManager,
							 const std::vector<RDFLiteralPtr> &negativeLiterals)
		: AnswerBroadcaster(),
		  kg_(kg),
		  reasonerManager_(reasonerManager),
		  negativeLiterals_(negativeLiterals) {
}

void NegationStage::pushToBroadcast(const AnswerPtr &msg) {
	if (isEOS(msg) || succeeds(msg)) {
		AnswerBroadcaster::pushToBroadcast(msg);
	}
}

bool NegationStage::succeeds(const AnswerPtr &answer) {
	// here the idea is to issue individual queries for each negative literal.
	// each query produces an AnswerBufferPtr object.
	// the set of all results can then be checked for a certain property, e.g. that none or all
	// of the queries produced a result.
	std::vector<AnswerBufferPtr> results;

	for (auto &lit: negativeLiterals_) {
		auto lit1 = lit->applySubstitution(*answer->substitution());
		// for now evaluate positive variant of the literal.
		// NOTE: for open-world semantics this cannot be done. open-world reasoner
		//       would need to receive negative literal instead.
		lit1->setIsNegated(false);

		// create an instance of the literal based on given substitution
		auto instance = RDFLiteral::fromLiteral(lit1);

		// check if the EDB contains positive lit, if so negation cannot be true
		results.push_back(kg_->submitQuery(
				std::make_shared<GraphQuery>(instance, QUERY_FLAG_ALL_SOLUTIONS)));

		// next check if positive lit is an IDB predicate, if so negation cannot be true.
		// get list of reasoner that define the literal
		// TODO: could be done faster through interface in manager
		std::vector<std::shared_ptr<Reasoner>> l_reasoner;
		for (auto &pair: reasonerManager_->reasonerPool()) {
			auto &r = pair.second->reasoner();
			if (r->canEvaluate(*instance)) {
				results.push_back(r->submitQuery(instance, QUERY_FLAG_ALL_SOLUTIONS));
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
