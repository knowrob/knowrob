/*
 * Copyright (c) 2024, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/interfaces/Interface.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/PastModality.h"

namespace knowrob {

	Interface::Interface(const boost::property_tree::ptree &config)
			: kb_(config)
	{

	}

	GraphAnswerFields Interface::processAskAllQuery(GraphQueryFields query) {
		// Implement your action here
		FormulaPtr phi(QueryParser::parse(query.queryString));

		FormulaPtr mPhi = applyModality(query, phi);

		auto resultStream = kb_.submitQuery(mPhi, QUERY_FLAG_ALL_SOLUTIONS);
		auto resultQueue = resultStream->createQueue();

		int numSolutions_ = 0;
		GraphAnswerFields result;
		while(true) {
			auto nextResult = resultQueue->pop_front();

			if(AnswerStream::isEOS(nextResult)) {
				break;
			}
			else {
				if (nextResult->substitution()->empty()) {
					numSolutions_ = 1;
					break;
				} else {
					// Push one answer
					GraphAnswerFields answer = createGraphAnswer(nextResult);
					result.answer.push_back(answer);
					numSolutions_ += 1;
					// publish feedback
					askallFeedback feedback;
					feedback.numberOfSolutions = numSolutions_;
					askall_action_server_.publishFeedback(feedback);
				}
			}
		}
	}

	FormulaPtr Interface::applyModality(const GraphQueryFields &query,
								FormulaPtr phi) {
		FormulaPtr mFormula = std::move(phi);

		// Add epistemic operator
		if (query.epistemicOperator == GraphQueryFields::BELIEF) {
			if (!query.aboutAgentIRI.empty()) {
				if (query.confidence != 1.0){
					mFormula = std::make_shared<ModalFormula>(
							BeliefModality::B(query.aboutAgentIRI, query.confidence), mFormula);
				} else {
					mFormula = std::make_shared<ModalFormula>(
							BeliefModality::B(query.aboutAgentIRI), mFormula);
				}
			}
		} else if (query.epistemicOperator == GraphQueryFields::KNOWLEDGE) {
			if (!query.aboutAgentIRI.empty()) {
				mFormula = std::make_shared<ModalFormula>(
						KnowledgeModality::K(query.aboutAgentIRI),mFormula);
			}
		}
		// Add temporal operator
		if (query.temporalOperator == GraphQueryFields::SOME_PAST) {
			if (query.minPastTimestamp != GraphQueryFields::UNSPECIFIED_TIMESTAMP
				|| query.maxPastTimestamp != GraphQueryFields::UNSPECIFIED_TIMESTAMP) {
				if (query.minPastTimestamp == GraphQueryFields::UNSPECIFIED_TIMESTAMP) {
					mFormula = std::make_shared<ModalFormula>(
							PastModality::P(TimeInterval(std::nullopt,
														 query.maxPastTimestamp)),mFormula);
				} else if (query.maxPastTimestamp == GraphQueryFields::UNSPECIFIED_TIMESTAMP) {
					mFormula = std::make_shared<ModalFormula>(
							PastModality::P(TimeInterval(query.minPastTimestamp,
														 std::nullopt)),mFormula);
				} else {
					mFormula = std::make_shared<ModalFormula>(
							PastModality::P(TimeInterval(query.minPastTimestamp,
														 query.maxPastTimestamp)),mFormula);
				}
			} else {
				mFormula = std::make_shared<ModalFormula>(
						PastModality::P(),mFormula);
			}
		} else if (query.temporalOperator == GraphQueryFields::ALL_PAST) {
			if (query.minPastTimestamp != GraphQueryFields::UNSPECIFIED_TIMESTAMP
				|| query.maxPastTimestamp != GraphQueryFields::UNSPECIFIED_TIMESTAMP) {
				if (query.minPastTimestamp == GraphQueryFields::UNSPECIFIED_TIMESTAMP) {
					mFormula = std::make_shared<ModalFormula>(
							PastModality::H(TimeInterval(std::nullopt,
														 query.maxPastTimestamp)),mFormula);
				} else if (query.maxPastTimestamp == GraphQueryFields::UNSPECIFIED_TIMESTAMP) {
					mFormula = std::make_shared<ModalFormula>(
							PastModality::H(TimeInterval(query.minPastTimestamp,
														 std::nullopt)),mFormula);
				} else {
					mFormula = std::make_shared<ModalFormula>(
							PastModality::H(TimeInterval(query.minPastTimestamp,
														 query.maxPastTimestamp)),mFormula);
				}
			} else {
				mFormula = std::make_shared<ModalFormula>(
						PastModality::H(),mFormula);
			}
		}
		return mFormula;
	}

}
