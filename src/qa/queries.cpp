/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */
 
// SWI Prolog
#include <SWI-Prolog.h>
// KnowRob
#include <knowrob/qa/queries.h>

using namespace knowrob;

/******************************************/
/*************** Formulae *****************/
/******************************************/

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

/******************************************/
/************* query results **************/
/******************************************/

const QueryResult* QueryResult::NO_SOLUTION = new QueryResult();

QueryResult::QueryResult(){}

bool QueryResult::hasSolution()
{
	return this != QueryResult::NO_SOLUTION;
}

void QueryResult::set(const Variable &var, const std::shared_ptr<Term> &term)
{
	mapping_[var] = term;
}

std::shared_ptr<Term> QueryResult::get(const Variable &var) const
{
	auto it = mapping_.find(var);
	if(it != mapping_.end()) {
		return it->second;
	}
	else {
		return std::shared_ptr<Term>();
	}
}

