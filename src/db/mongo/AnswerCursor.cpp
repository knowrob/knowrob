/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/predicate.hpp>
#include "knowrob/Logger.h"
#include "knowrob/db/mongo/AnswerCursor.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"

using namespace knowrob;
using namespace knowrob::mongo;

AnswerCursor::AnswerCursor(const std::shared_ptr<Collection> &collection)
		: Cursor(collection),
		  resultDocument_(nullptr),
		  resultIter_(),
		  timeIter_(),
		  varIter_(),
		  valIter_(),
		  scopeIter_() {
}

void AnswerCursor::setSubstitution(const std::shared_ptr<AnswerYes> &answer) {
	while (bson_iter_next(&varIter_)) {
		auto var = std::make_shared<Variable>(bson_iter_key(&varIter_));

		if (!bson_iter_recurse(&varIter_, &valIter_)) continue;
		if (!bson_iter_find(&valIter_, "val")) continue;

		// read the value of the variable
		switch (bson_iter_type(&valIter_)) {
			case BSON_TYPE_UTF8: {
				auto utf8 = bson_iter_utf8(&valIter_, nullptr);
				// note: currently mongo KG does not store the type of the literal,
				// so we cannot trivially distinguish between IRI and literal and need to guess here.
				switch (rdfNodeTypeGuess(utf8)) {
					case RDFNodeType::BLANK:
						answer->set(var, std::make_shared<Blank>(utf8));
						break;
					case RDFNodeType::IRI:
						answer->set(var, std::make_shared<IRIAtom>(utf8));
						break;
					case RDFNodeType::LITERAL:
						answer->set(var, std::make_shared<StringView>(utf8));
						break;
				}
				break;
			}
			case BSON_TYPE_INT32:
				answer->set(var, std::make_shared<Integer>(bson_iter_int32(&valIter_)));
				break;
			case BSON_TYPE_INT64:
				answer->set(var, std::make_shared<Long>(bson_iter_int64(&valIter_)));
				break;
			case BSON_TYPE_BOOL:
				answer->set(var, std::make_shared<Integer>(bson_iter_bool(&valIter_)));
				break;
			case BSON_TYPE_DOUBLE:
				answer->set(var, std::make_shared<Double>(bson_iter_double(&valIter_)));
				break;
			default:
				KB_WARN("unsupported type {} for predicate arguments.", bson_iter_type(&valIter_));
				break;
		}
	}
}

std::shared_ptr<GraphSelector> AnswerCursor::readAnswerFrame(const std::shared_ptr<AnswerYes> &answer) {
	std::shared_ptr<GraphSelector> frame;

	while (bson_iter_next(&scopeIter_)) {
		std::string_view scopeKey(bson_iter_key(&scopeIter_));

		if (scopeKey == "uncertain") {
			if (!frame) frame = std::make_shared<GraphSelector>();
			if (bson_iter_bool(&scopeIter_)) {
				frame->epistemicOperator = EpistemicOperator::BELIEF;
			}
		} else if (scopeKey == "time" && bson_iter_recurse(&scopeIter_, &timeIter_)) {
			if (!frame) frame = std::make_shared<GraphSelector>();
			std::optional<TimePoint> since, until;
			while (bson_iter_next(&timeIter_)) {
				std::string_view timeKey(bson_iter_key(&timeIter_));
				if (timeKey == "since") {
					frame->begin = bson_iterOptionalDouble(&timeIter_);
					if (frame->begin.value() == 0) {
						frame->begin = std::nullopt;
					}
				} else if (timeKey == "until") {
					frame->end = bson_iterOptionalDouble(&timeIter_);
				}
			}
		} else if (scopeKey == "confidence") {
			if (!frame) frame = std::make_shared<GraphSelector>();
			frame->confidence = bson_iter_double(&timeIter_);
			if (frame->confidence.value() < 0.999) {
				frame->epistemicOperator = EpistemicOperator::BELIEF;
			} else {
				frame->epistemicOperator = EpistemicOperator::KNOWLEDGE;
			}
		} else if (scopeKey == "agent") {
			if (!frame) frame = std::make_shared<GraphSelector>();
			auto agent_iri = bson_iter_utf8(&timeIter_, nullptr);
			if (!frame->epistemicOperator.has_value()) {
				frame->epistemicOperator = EpistemicOperator::KNOWLEDGE;
			}
			frame->agent = Agent::get(agent_iri);
		} else if (scopeKey == "occasional") {
			if (!frame) frame = std::make_shared<GraphSelector>();
			if (bson_iter_bool(&scopeIter_)) {
				frame->temporalOperator = TemporalOperator::SOMETIMES;
			} else {
				frame->temporalOperator = TemporalOperator::ALWAYS;
			}
		}
		// TODO: what about "ontology"/"graph" field?
	}

	if (frame) {
		return frame;
	} else {
		return {};
	}
}

bool AnswerCursor::nextAnswer(
		const std::shared_ptr<AnswerYes> &answer,
		const std::vector<FramedTriplePatternPtr> &literals) {
	if (!next(&resultDocument_)) return false;
	if (!bson_iter_init(&resultIter_, resultDocument_)) return false;

	std::shared_ptr<GraphSelector> frame_rw;
	while (bson_iter_next(&resultIter_)) {
		std::string_view resultKey(bson_iter_key(&resultIter_));
		// read variables from "v_VARS". each sub-field is named according to the field of
		// a variable.
		if (resultKey == "v_VARS" && bson_iter_recurse(&resultIter_, &varIter_)) {
			setSubstitution(answer);
		} else if (resultKey == "v_scope" && bson_iter_recurse(&resultIter_, &scopeIter_)) {
			frame_rw = readAnswerFrame(answer);
		}
	}

	GraphSelectorPtr frame_readonly;
	if (frame_rw) {
		frame_readonly = frame_rw;
		answer->setFrame(frame_rw);
	} else {
		frame_readonly = DefaultGraphSelector();
	}

	// add predicate groundings to the answer
	for (auto &rdfLiteral: literals) {
		auto p = rdfLiteral->predicate();
		auto p_instance = applyBindings(p, *answer->substitution());
		answer->addGrounding(
				std::static_pointer_cast<Predicate>(p_instance),
				frame_readonly,
				rdfLiteral->isNegated());
	}

	return true;
}
