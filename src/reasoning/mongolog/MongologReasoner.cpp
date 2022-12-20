/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/reasoning/mongolog/MongologReasoner.h>

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

	// TODO: load mongolog code into Prolog
	// TODO: properly handle path here
	consult("src/reasoning/mongolog/mongolog/__init__.pl");

	return PrologReasoner::initialize(cfg);
}

bool MongologReasoner::canReasonAbout(const PredicateIndicator &predicate)
{
	// TODO call is_mongolog_predicate
	return false;
}

std::shared_ptr<Query> MongologReasoner::transformQuery(const std::shared_ptr<Query> &q)
{
	static const auto indicator = std::make_shared<PredicateIndicator>("mongolog_call",1);
	
	// wrap queries in `mongolog_call/1`
	auto t = PrologQuery::toTerm(q->formula());
	return std::make_shared<Query>(
		std::make_shared<Predicate>(indicator, std::vector<TermPtr>{t}));
}

