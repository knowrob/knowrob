/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/mongolog/MongologReasoner.h>

using namespace knowrob;

MongologReasoner::MongologReasoner(const std::string &reasonerID)
: PrologReasoner(reasonerID)
{}

MongologReasoner::~MongologReasoner()
= default;

bool MongologReasoner::initializeDefaultPackages()
{
	static bool initialized = false;

	if(initialized) {
		return true;
	} else {
		initialized = true;
		// load mongolog code once globally into the Prolog engine
		return consultIntoUser(std::filesystem::path("mongolog") / "__init__.pl");
	}
}

bool MongologReasoner::isCurrentPredicate(const PredicateIndicator &predicate)
{
	// TODO: better map to current_predicate as regular mongolog query?
	return !QueryResultStream::isEOS(oneSolution1(std::make_shared<Query>(
		std::make_shared<Predicate>(Predicate(
			"is_mongolog_predicate", {
				std::make_shared<StringTerm>(predicate.functor()) //,
				//std::make_shared<Integer32Term>(predicate.arity())
			}
		))
	)));
}

std::shared_ptr<Query> MongologReasoner::transformQuery(const std::shared_ptr<Query> &q)
{
	static const auto indicator = std::make_shared<PredicateIndicator>("mongolog_call",1);
	
	// wrap queries in `mongolog_call/1`
	auto t = PrologQuery::toTerm(q->formula());
	return std::make_shared<Query>(
		std::make_shared<Predicate>(indicator, std::vector<TermPtr>{t}));
}
