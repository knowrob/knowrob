/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Predicate.h"
#include "knowrob/terms/Constant.h"

using namespace knowrob;

namespace knowrob {
	PredicateType predicateTypeFromTerm(const TermPtr &term) {
		auto type_string = ((StringTerm*)term.get())->value();
		if(type_string == "relation") return PredicateType::RELATION;
		else                          return PredicateType::BUILT_IN;
	}
}

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


Predicate::Predicate(
	const std::shared_ptr<PredicateIndicator> &indicator,
	const std::vector<TermPtr> &arguments)
: Term(TermType::PREDICATE),
  indicator_(indicator),
  arguments_(arguments),
  variables_(getVariables1())
{
}

Predicate::Predicate(const Predicate &other, const Substitution &sub)
: Term(TermType::PREDICATE),
  indicator_(other.indicator_),
  arguments_(applySubstitution(other.arguments_, sub)),
  variables_(getVariables1())
{
}

Predicate::Predicate(
	const std::string &functor,
	const std::vector<TermPtr> &arguments)
: Predicate(std::make_shared<PredicateIndicator>(functor, arguments.size()), arguments)
{
}

bool Predicate::isEqual(const Term& other) const {
	const auto &x = static_cast<const Predicate&>(other); // NOLINT
    if(*indicator() == *x.indicator() && arguments_.size() == x.arguments_.size()) {
        for(int i=0; i<arguments_.size(); ++i) {
            if(!(*(arguments_[i]) == *(x.arguments_[i]))) return false;
        }
        return true;
    }
    else {
        return false;
    }
}

VariableSet Predicate::getVariables1() const
{
	VariableSet out;
	for(auto &arg : arguments_) {
		for(auto &var : arg->getVariables()) {
			out.insert(var);
		}
	}
	return out;
}

std::vector<TermPtr> Predicate::applySubstitution(
	const std::vector<TermPtr> &in,
	const Substitution &sub)
{
	std::vector<TermPtr> out(in.size());
	
	for(uint32_t i=0; i<in.size(); i++) {
		switch(in[i]->type()) {
		case TermType::VARIABLE: {
			// replace variable argument if included in the substitution mapping
			const TermPtr& t = sub.get(*((Variable*) in[i].get()));
			if(t) {
				// replace variable with term
				out[i] = t;
			} else {
				// variable is not included in the substitution, keep it
				out[i] = in[i];
			}
			break;
		}
		case TermType::PREDICATE: {
			// recursively apply substitution
			auto *p = (Predicate*)in[i].get();
			out[i] = (p->isGround() ?
				in[i] :
				p->applySubstitution(sub));
			break;
		}
		default:
			out[i] = in[i];
			break;
		}
	}
	
	return out;
}

std::shared_ptr<Predicate> Predicate::applySubstitution(const Substitution &sub) const
{
	return std::make_shared<Predicate>(*this, sub);
}

void Predicate::write(std::ostream& os) const
{
	// TODO: some predicates should be written in infix notation, e.g. '=', 'is', ...
	os << indicator_->functor();
	if(!arguments_.empty()) {
		os << '(';
		for(uint32_t i=0; i<arguments_.size(); i++) {
			Term *t = arguments_[i].get();
			os << (*t);
			if(i+1 < arguments_.size()) {
				os << ',' << ' ';
			}
		}
		os << ')';
	}
}
