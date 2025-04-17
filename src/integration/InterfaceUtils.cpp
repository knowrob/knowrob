/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <unordered_map>
#include <boost/any.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "knowrob/integration/InterfaceUtils.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/semweb/Triple.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/KnowledgeBase.h"


using namespace knowrob;

// enum for epistemicOperator
enum EpistemicOperator {
	KNOWLEDGE = 0,
	BELIEF = 1
};

// enum for temporalOperator
enum TemporalOperator {
	CURRENTLY = 0,
	ALL_PAST = 1,
	SOME_PAST = 2
};

boost::property_tree::ptree InterfaceUtils::loadSettings() {
	// Check for settings file
	std::string config_path = "default.json";
	if (std::getenv("KNOWROB_SETTINGS")) {
		config_path = std::getenv("KNOWROB_SETTINGS");
	}

	// read the settings
	boost::property_tree::ptree config;
	boost::property_tree::read_json(
			config_path,
			config);

	return config;

}

bool InterfaceUtils::assertStatements(const KnowledgeBasePtr &kb_, const std::vector<FormulaPtr> &args) {
	// Create the query trees for each formula and calculate the number of nodes of type Predicate
	// first create the list of query trees
	std::vector<std::unique_ptr<QueryTree>> queryTrees;
	for (auto &phi : args) {
		auto qt = std::make_unique<QueryTree>(phi);
		if (qt->numPaths() > 1) {
			throw QueryError("Disjunctions are not allowed in assertions. "
							 "Appears in statement {}.", *phi);
		} else if (qt->numPaths() == 0) {
			throw QueryError("Invalid assertion: '{}'", *phi);
		}
		queryTrees.push_back(std::move(qt));  // ✔️ move the unique_ptr
	}
	// create a counter for the number of nodes of type Predicate
	int numPredicates = 0;
	for (auto &qt: queryTrees) {
		for (auto &psi: qt->begin()->nodes()) {
			if (psi->type() == knowrob::FormulaType::PREDICATE) {
				numPredicates += 1;
			}
		}
	}
	
	// create a vector of TriplePtr and TriplePatternPtr
	std::vector<TriplePtr> data(numPredicates);
	std::vector<TriplePatternPtr> buf(numPredicates);
	uint32_t dataIndex = 0;

	// iterate over the query trees and create the TriplePattern objects
	for (auto &qt: queryTrees) {
		for (auto &psi: qt->begin()->nodes()) {
			switch (psi->type()) {
				case knowrob::FormulaType::PREDICATE:
					buf[dataIndex] = std::make_shared<TriplePattern>(
							std::static_pointer_cast<Predicate>(psi), false);
					data[dataIndex].ptr = new TripleCopy();
					data[dataIndex].owned = true;
					buf[dataIndex]->instantiateInto(*data[dataIndex].ptr);
					dataIndex += 1;
					break;
				default:
					throw QueryError("Invalid assertion: '{}'", *psi);
			}
		}
	}


	if (kb_->insertAll(data)) {
		std::cout << "success, " << dataIndex << " statement(s) were asserted." << "\n";
		return true;
	} else {
		std::cout << "assertion failed." << "\n";
		return false;
	}
}

FormulaPtr
InterfaceUtils::applyModality(const std::unordered_map<std::string, boost::any> &options,
							  FormulaPtr phi) {
	FormulaPtr mFormula = std::move(phi);

	// Retrieve epistemicOperator and check if it is "BELIEF"
	auto epistemicOperator = boost::any_cast<int>(options.at("epistemicOperator"));
	if (epistemicOperator == EpistemicOperator::BELIEF) {
		// Retrieve aboutAgentIRI and confidence
		auto aboutAgentIRI = boost::any_cast<std::string>(options.at("aboutAgentIRI"));
		auto confidence = boost::any_cast<double>(options.at("confidence"));
		if (!aboutAgentIRI.empty()) {
			if (confidence != 1.0) {
				mFormula = std::make_shared<ModalFormula>(
						modals::B(aboutAgentIRI, confidence), mFormula);
			} else {
				mFormula = std::make_shared<ModalFormula>(
						modals::B(aboutAgentIRI), mFormula);
			}
		}
	} else if (epistemicOperator == EpistemicOperator::KNOWLEDGE) {
		// Retrieve aboutAgentIRI
		auto aboutAgentIRI = boost::any_cast<std::string>(options.at("aboutAgentIRI"));
		if (!aboutAgentIRI.empty()) {
			mFormula = std::make_shared<ModalFormula>(
					modals::K(aboutAgentIRI), mFormula);
		}
	}
	// Retrieve temporalOperator
	auto temporalOperator = boost::any_cast<int>(options.at("temporalOperator"));

	// Retrieve minPastTimestamp and maxPastTimestamp
	auto minPastTimestamp = boost::any_cast<double>(options.at("minPastTimestamp"));
	auto maxPastTimestamp = boost::any_cast<double>(options.at("maxPastTimestamp"));

	auto minPastTimePoint =
			minPastTimestamp != -1 ? std::optional<TimePoint>(knowrob::time::fromSeconds(minPastTimestamp))
								   : std::nullopt;
	auto maxPastTimePoint =
			maxPastTimestamp != -1 ? std::optional<TimePoint>(knowrob::time::fromSeconds(maxPastTimestamp))
								   : std::nullopt;

	if (temporalOperator == TemporalOperator::SOME_PAST) {
		if (minPastTimestamp != -1 || maxPastTimestamp != -1) {
			if (minPastTimestamp == -1) {
				mFormula = std::make_shared<ModalFormula>(
						modals::P(TimeInterval(std::nullopt,
											   maxPastTimePoint)), mFormula);
			} else if (maxPastTimestamp == -1) {
				mFormula = std::make_shared<ModalFormula>(
						modals::P(TimeInterval(minPastTimePoint,
											   std::nullopt)), mFormula);
			} else {
				mFormula = std::make_shared<ModalFormula>(
						modals::P(TimeInterval(minPastTimePoint,
											   maxPastTimePoint)), mFormula);
			}
		} else {
			mFormula = std::make_shared<ModalFormula>(
					modals::P(), mFormula);
		}
	} else if (temporalOperator == TemporalOperator::ALL_PAST) {
		if (minPastTimestamp != -1 || maxPastTimestamp != -1) {
			if (minPastTimestamp == -1) {
				mFormula = std::make_shared<ModalFormula>(
						modals::H(TimeInterval(std::nullopt,
											   maxPastTimePoint)), mFormula);
			} else if (maxPastTimestamp == -1) {
				mFormula = std::make_shared<ModalFormula>(
						modals::H(TimeInterval(minPastTimePoint,
											   std::nullopt)), mFormula);
			} else {
				mFormula = std::make_shared<ModalFormula>(
						modals::H(TimeInterval(minPastTimePoint,
											   maxPastTimePoint)), mFormula);
			}
		} else {
			mFormula = std::make_shared<ModalFormula>(
					modals::H(), mFormula);
		}
	}
	return mFormula;
}

namespace knowrob::py {
	template<>
	void createType<InterfaceUtils>() {
		using namespace boost::python;

		// Expose InterfaceUtils class and its methods
		class_<InterfaceUtils>("InterfaceUtils")
				.def("assertStatements", &InterfaceUtils::assertStatements).staticmethod("assertStatements")
				.def("applyModality", &InterfaceUtils::applyModality).staticmethod("applyModality");

		// Expose enums
		enum_<EpistemicOperator>("EpistemicOperator")
				.value("KNOWLEDGE", KNOWLEDGE)
				.value("BELIEF", BELIEF);
		enum_<TemporalOperator>("TemporalOperator")
				.value("CURRENTLY", CURRENTLY)
				.value("ALL_PAST", ALL_PAST)
				.value("SOME_PAST", SOME_PAST);
	}
}
