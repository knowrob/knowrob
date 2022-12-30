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

MongologReasoner::MongologReasoner()
: PrologReasoner()
{}

MongologReasoner::~MongologReasoner()
{
}

bool MongologReasoner::initialize(const ReasonerConfiguration &cfg)
{
	// TODO: initialize database connection
	
	if(!initializeDefaultPackages(cfg)) {
		return false;
	}

	// load mongolog code into Prolog
	// TODO: properly handle path here
	if(!consult1("src/mongolog/__init__.pl")) {
		return false;
	}

	return initializeConfiguration(cfg);
}

bool MongologReasoner::canReasonAbout(const PredicateIndicator &predicate)
{
	// TODO: could be cached, or initially loaded into a set
	return !QueryResultStream::isEOS(oneSolution(std::make_shared<Query>(
		std::shared_ptr<Predicate>(new Predicate(
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
