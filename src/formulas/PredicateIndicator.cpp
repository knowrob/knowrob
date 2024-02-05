/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/formulas/PredicateIndicator.h"
#include "knowrob/terms/Constant.h"

using namespace knowrob;

PredicateIndicator::PredicateIndicator(std::string functor, unsigned int arity)
: functor_(std::move(functor)),
  arity_(arity)
{
}

bool PredicateIndicator::operator==(const PredicateIndicator& other) const
{
    return arity_ == other.arity_ && functor_ == other.functor_;
}

bool PredicateIndicator::operator< (const PredicateIndicator& other) const
{
	return (other.functor_ < this->functor_) ||
	       (other.arity_   < this->arity_);
}

void PredicateIndicator::write(std::ostream& os) const
{
	os << functor_ << '/' << arity_;
}

std::shared_ptr<Term> PredicateIndicator::toTerm() const
{
	static const auto indicatorIndicator = std::make_shared<PredicateIndicator>("/",2);
	return std::make_shared<Predicate>(Predicate(indicatorIndicator, {
		std::make_shared<StringTerm>(functor()),
		std::make_shared<LongTerm>(arity())
	}));
}
