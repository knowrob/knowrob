/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/ComputableReasoner.h"


namespace knowrob {

	std::shared_ptr<PredicateDescription> ComputableReasoner::getPredicateDescription(
			const std::shared_ptr<PredicateIndicator> &indicator) {
		if(functors.count(indicator->functor())) {
			return std::make_shared<PredicateDescription>(
					std::make_shared<PredicateIndicator>(indicator->functor(),2), PredicateType::RELATION);
		}
		else {
			return {};
		}
	}

	AnswerBufferPtr ComputableReasoner::submitQuery(const RDFLiteralPtr &literal, int queryFlags) {
		bool sendEOS = true;
		auto answerBuffer = std::make_shared<AnswerBuffer>();
		auto outputChannel = AnswerStream::Channel::create(answerBuffer);

		auto p = std::static_pointer_cast<StringTerm>(literal->propertyTerm()) ;

		computableFunc f = functors[p->value()];
		(*f)(literal, outputChannel);

		outputChannel->push(AnswerStream::eos());
		return answerBuffer;
	}



} // knowrob