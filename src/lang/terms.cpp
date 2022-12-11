/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/lang/terms.h>

using namespace knowrob;

Term::Term(TermType type)
: type_(type)
{}

std::ostream& operator<<(std::ostream& os, const Term& t)
{
	t.write(os);
	return os;
}


Variable::Variable(const std::string &name)
: Term(TermType::VARIABLE),
  name_(name)
{}

bool Variable::operator< (const Variable& other) const
{
	return (this->name_ < other.name_);
}

void Variable::write(std::ostream& os) const
{
	os << "var" << '(' << name_ << ')';
}


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


PredicateIndicator::PredicateIndicator(const std::string &functor, unsigned int arity)
: functor_(functor),
  arity_(arity)
{}

bool PredicateIndicator::operator< (const PredicateIndicator& other) const
{
	return (other.functor_ < this->functor_) ||
	       (other.arity_   < this->arity_);
}

void PredicateIndicator::write(std::ostream& os) const
{
	os << functor_ << '/' << arity_;
}


Predicate::Predicate(
	const std::shared_ptr<PredicateIndicator> &indicator,
	const std::vector<std::shared_ptr<Term>> &arguments)
: Term(TermType::PREDICATE),
  indicator_(indicator),
  arguments_(arguments),
  isGround_(isGround1())
{
}

Predicate::Predicate(const Predicate &other, const Substitution &sub)
: Term(TermType::PREDICATE),
  indicator_(other.indicator_),
  arguments_(applySubstitution(other.arguments_, sub)),
  isGround_(isGround1())
{
}

Predicate::Predicate(
	const std::string &functor,
	const std::vector<std::shared_ptr<Term>> &arguments)
: Predicate(
	std::shared_ptr<PredicateIndicator>(
		new PredicateIndicator(functor, arguments.size())
	),
	arguments)
{
}

bool Predicate::isGround1() const
{
	for(const auto &x : arguments_) {
		if(!x->isGround()) return false;
	}
	return true;
}

std::vector<std::shared_ptr<Term>> Predicate::applySubstitution(
	const std::vector<std::shared_ptr<Term>> &in,
	const Substitution &sub) const
{
	std::vector<std::shared_ptr<Term>> out(in.size());
	
	for(uint32_t i=0; i<in.size(); i++) {
		switch(in[i]->type()) {
		case TermType::VARIABLE: {
			// replace variable argument if included in the substitution mapping
			std::shared_ptr<Term> t = sub.get(*((Variable*) in[i].get()));
			if(t.get() == NULL) {
				// variable is not included in the substitution, keep it
				out[i] = in[i];
			} else {
				// replace variable with term
				out[i] = t;
			}
			break;
		}
		case TermType::PREDICATE: {
			// recursively apply substitution
			Predicate *p = (Predicate*)in[i].get();
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
	return std::shared_ptr<Predicate>(new Predicate(*this, sub));
}

void Predicate::write(std::ostream& os) const
{
	os << indicator_->functor() << '(';
	for(uint32_t i=0; i<arguments_.size(); i++) {
		Term *t = arguments_[i].get();
		os << (*t);
		if(i+1 < arguments_.size()) {
			os << ',' << ' ';
		}
	}
	os << ')';
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

std::shared_ptr<Substitution> Substitution::combine(
	const std::vector<std::shared_ptr<Substitution>> &subs)
{
	static const std::shared_ptr<Substitution> null_sub;
	auto combined = std::shared_ptr<Substitution>(new Substitution);
	
	// get all instantiations of each variable
	std::map<Variable, std::set<std::shared_ptr<Term>>> instantiations;
	for(const auto &sub : subs) {
		for(const auto &pair : sub->mapping_) {
			instantiations[pair.first].insert(pair.second);
		}
	}
	
	// for each variable with multiple instances, unify the instances.
	// if unification fails, return null_sub.
	std::shared_ptr<Term> unifiedInstance;
	for(const auto &pair : instantiations) {
		if(pair.second.size()==1) {
			// variable has only one instantiation
			combined->set(pair.first, *(pair.second.begin()));
		}
		else {
			// TODO multiple instantiations need to be unified
			// unifiedInstance = Term::unify(pair.second);
			if(unifiedInstance.get()==NULL) {
				// unification not possible
				return null_sub;
			}
			else {
				combined->set(pair.first, unifiedInstance);
			}
		}
	}
	
	return combined;
}

