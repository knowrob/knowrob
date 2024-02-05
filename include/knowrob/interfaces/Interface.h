/*
 * Copyright (c) 2024, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_INTERFACE_H
#define KNOWROB_INTERFACE_H

#include <boost/property_tree/ptree.hpp>
#include "knowrob/formulas/Formula.h"
#include "knowrob/KnowledgeBase.h"

namespace knowrob {
	struct GraphQueryFields {
		// Constants
		static const uint8_t LANG_FOL = 0;
		static const uint8_t KNOWLEDGE = 0;
		static const uint8_t BELIEF = 1;
		static const uint8_t CURRENTLY = 0;
		static const uint8_t ALL_PAST = 1;
		static const uint8_t SOME_PAST = 2;
		static const double UNSPECIFIED_TIMESTAMP;

		// Message fields
		std::string lang = std::to_string(LANG_FOL); // Default: LANG_FOL
		std::string queryString;
		uint8_t epistemicOperator = KNOWLEDGE; // Default: KNOWLEDGE
		std::string aboutAgentIRI = ""; // Default: ""
		std::string aboutSimulationIRI = ""; // Default: ""
		uint8_t temporalOperator = CURRENTLY; // Default: CURRENTLY
		double minPastTimestamp = UNSPECIFIED_TIMESTAMP; // Default: UNSPECIFIED_TIMESTAMP
		double maxPastTimestamp = UNSPECIFIED_TIMESTAMP; // Default: UNSPECIFIED_TIMESTAMP
		double confidence = 0.0; // Default: 0.0
	};

	struct KeyValuePairFields {
		// Constants
		static const int32_t TYPE_STRING = 0;
		static const int32_t TYPE_FLOAT = 1;
		static const int32_t TYPE_INT = 2;
		static const int32_t TYPE_LONG = 3;
		static const int32_t TYPE_VARIABLE = 4;
		static const int32_t TYPE_PREDICATE = 5;
		static const int32_t TYPE_LIST = 6;

		// Message fields
		int32_t type;
		std::string key;
		std::string value_string;
		double value_float;
		int32_t value_int;
		int64_t value_long;
		std::string value_variable;
		std::string value_predicate;
		std::string value_list;
	};

	struct GraphAnswerFields {
		std::vector<KeyValuePairFields> substitution;
		std::string instantiation;
		double confidence;
		double startTimestamp;
		double endTimestamp;
	};

	class Interface {
	private:


		KnowledgeBase kb_;
	public:
		Interface() = default;

		virtual ~Interface() = default;

		Interface(const boost::property_tree::ptree &config);

		virtual void load(const boost::property_tree::ptree &config) = 0;

		GraphAnswerFields processAskAllQuery(GraphQueryFields query);

		FormulaPtr applyModality(const GraphQueryFields &query,
								 FormulaPtr phi);

	};
}


#endif //KNOWROB_INTERFACE_H
