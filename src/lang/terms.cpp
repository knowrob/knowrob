/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/lang/terms.h>

using namespace knowrob;

StringTerm::StringTerm(const std::string &v)
: Constant(TermType::STRING, v)
{}

DoubleTerm::DoubleTerm(const double &v)
: Constant(TermType::DOUBLE, v)
{}
	
LongTerm::LongTerm(const long &v)
: Constant(TermType::LONG, v)
{}
	
Integer32Term::Integer32Term(const int32_t &v)
: Constant(TermType::INT32, v)
{}

Predicate::Predicate(const std::string &functor, const std::vector<std::shared_ptr<Term>> &arguments)
: Term(TermType::PREDICATE),
  indicator_(functor, arguments.size()),
  arguments_(arguments),
  hasFreeVariable_(false)
{
	for(const auto &x : arguments) {
		if(x->hasFreeVariable()) hasFreeVariable_ = true;
	}
}

void Predicate::applySubstitution(const Substitution &sub)
{
	// TODO
}


void Substitution::set(const Variable &var, const std::shared_ptr<Term> &term)
{
	mapping_[var] = term;
}

bool Substitution::contains(const Variable &var) const
{
	return mapping_.find(var) != mapping_.end();
}

std::shared_ptr<Term> Substitution::get(const Variable &var) const
{
	static const std::shared_ptr<Term> null_term;
	
	auto it = mapping_.find(var);
	if(it != mapping_.end()) {
		return it->second;
	}
	else {
		return null_term;
	}
}

