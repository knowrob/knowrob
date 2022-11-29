/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/qa/queries.h>

using namespace knowrob;

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

std::set<Variable> Formula::getVariables()
{
	std::set<Variable> out;
	readVariables(out);
	return out;
}

void PredicateFormula::readVariables(std::set<Variable> &out)
{
	// TODO: read p_ vars
}

void ConnectiveFormula::readVariables(std::set<Variable> &out)
{
	for(auto &phi : formulae_) {
		phi.readVariables(out);
	}
}

QueryResult::QueryResult(const boost::shared_ptr<Substitution> &substitution)
: hasSolution_(true),
  substitution_(substitution)
{}

QueryResult::QueryResult()
: hasSolution_(false)
{}

QueryResult::~QueryResult()
{}
		
static const QueryResult& QueryResult::noSolution()
{
	static QueryResult instance;
	return instance;
}

