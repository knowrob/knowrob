/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/lang/terms.h>

// TODO: support for fuzzy and temporalized predicates.

using namespace knowrob;

Term::Term(TermType type)
: type_(type)
{}

bool Term::isBottom() const
{
	return (this == BottomTerm::get().get());
}

bool Term::isTop() const
{
	return (this == TopTerm::get().get());
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
	os << name_;
}


StringTerm::StringTerm(const std::string &v)
: Constant(TermType::STRING, v)
{}

void StringTerm::write(std::ostream& os) const
{
	os << '\'' << value_ << '\'';
}

DoubleTerm::DoubleTerm(const double &v)
: Constant(TermType::DOUBLE, v)
{}
	
LongTerm::LongTerm(const long &v)
: Constant(TermType::LONG, v)
{}
	
Integer32Term::Integer32Term(const int32_t &v)
: Constant(TermType::INT32, v)
{}


ListTerm::ListTerm(const std::vector<TermPtr> &elements)
: Term(TermType::LIST),
  elements_(elements),
  isGround_(isGround1())
{
}

bool ListTerm::isGround1() const
{
	for(const auto &x : elements_) {
		if(!x->isGround()) return false;
	}
	return true;
}
		
std::shared_ptr<ListTerm> ListTerm::nil()
{
	static std::shared_ptr<ListTerm> x(new ListTerm(
		std::vector<std::shared_ptr<Term>>(0)));
	return x;
}

bool ListTerm::isNIL() const
{
	return elements_.empty();
}

void ListTerm::write(std::ostream& os) const
{
	os << '[';
	for(uint32_t i=0; i<elements_.size(); i++) {
		Term *t = elements_[i].get();
		os << (*t);
		if(i+1 < elements_.size()) {
			os << ',' << ' ';
		}
	}
	os << ']';
}


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
	const std::vector<TermPtr> &arguments)
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
	const std::vector<TermPtr> &arguments)
: Predicate(std::make_shared<PredicateIndicator>(functor, arguments.size()), arguments)
{
}

bool Predicate::isGround1() const
{
	for(const auto &x : arguments_) {
		if(!x->isGround()) return false;
	}
	return true;
}

std::vector<TermPtr> Predicate::applySubstitution(
	const std::vector<TermPtr> &in,
	const Substitution &sub) const
{
	std::vector<TermPtr> out(in.size());
	
	for(uint32_t i=0; i<in.size(); i++) {
		switch(in[i]->type()) {
		case TermType::VARIABLE: {
			// replace variable argument if included in the substitution mapping
			TermPtr t = sub.get(*((Variable*) in[i].get()));
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
	return std::make_shared<Predicate>(*this, sub);
}

void Predicate::write(std::ostream& os) const
{
	// TODO: some predicates can be written in infix notation
	os << indicator_->functor();
	if(arguments_.size()>0) {
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


const std::shared_ptr<TopTerm>& TopTerm::get()
{
	static std::shared_ptr<TopTerm> singleton(new TopTerm);
	return singleton;
}

TopTerm::TopTerm()
: Predicate("true", std::vector<TermPtr>())
{
}

void TopTerm::write(std::ostream& os) const
{
	os << "\u22A4";
}


const std::shared_ptr<BottomTerm>& BottomTerm::get()
{
	static std::shared_ptr<BottomTerm> singleton(new BottomTerm);
	return singleton;
}

BottomTerm::BottomTerm()
: Predicate("false", std::vector<TermPtr>())
{
}

void BottomTerm::write(std::ostream& os) const
{
	os << "\u22A5";
}


void Substitution::set(const Variable &var, const TermPtr &term)
{
	mapping_.insert(std::pair<Variable,TermPtr>(var, term));
}

bool Substitution::contains(const Variable &var) const
{
	return mapping_.find(var) != mapping_.end();
}

const TermPtr& Substitution::get(const Variable &var) const
{
	static const TermPtr null_term;
	
	auto it = mapping_.find(var);
	if(it != mapping_.end()) {
		return it->second;
	}
	else {
		return null_term;
	}
}

bool Substitution::combine(const std::shared_ptr<Substitution> &other, Substitution::Diff &changes)
{
	for(const auto &pair : other->mapping_) {
		auto it = mapping_.find(pair.first);
		if(it == mapping_.end()) {
			// new variable instantiation
			changes.push(std::shared_ptr<Operation>(
				new Added(mapping_.insert(pair).first)));
		}
		else {
			// variable has already an instantation, need to unify
			TermPtr t0 = it->second;
			TermPtr t1 = pair.second;
			
			// t0 and t1 are not syntactically equal -> compute a unifier
			Unifier sigma(t0,t1);
			if(sigma.exists()) {
				// a unifier exists
				it->second = sigma.apply();
				changes.push(std::shared_ptr<Operation>(new Replaced(it, t0)));
			}
			else {
				// no unifier exists
				return false;
			}
		}
	}
	
	return true;
}

void Substitution::rollBack(Substitution::Diff &changes)
{
	while(!changes.empty()) {
		auto &change = changes.front();
		change->rollBack(*this);
		changes.pop();
	}
}


Substitution::Replaced::Replaced(const Substitution::Iterator &it, const TermPtr &replacedInstance)
: Substitution::Operation(),
  it_(it),
  replacedInstance_(replacedInstance)
{}

void Substitution::Replaced::rollBack(Substitution &sub)
{
	// set old value
	it_->second = replacedInstance_;
}


Substitution::Added::Added(const Substitution::Iterator &it)
: Substitution::Operation(),
  it_(it)
{}

void Substitution::Added::rollBack(Substitution &sub)
{
	// remove the added variable instantiation
	sub.mapping_.erase(it_);
}


Unifier::Unifier(const TermPtr &t0, const TermPtr &t1)
: Substitution(),
  t0_(t0),
  t1_(t1),
  exists_(unify(t0,t1))
{
}

bool Unifier::unify(const TermPtr &t0, const TermPtr &t1)
{
	if(t1->type() == TermType::VARIABLE) {
		unify(*((Variable*)t1.get()), t0);
	}
	else {
		switch(t0->type()) {
		case TermType::VARIABLE:
			unify(*((Variable*)t0.get()), t1);
			break;
		case TermType::PREDICATE: {
			// predicates only unify with other predicates
			if(t1->type()!=TermType::PREDICATE) {
				return false;
			}
			Predicate *p0 = (Predicate*)t0.get();
			Predicate *p1 = (Predicate*)t1.get();
			// test for functor equality, and matching arity
			if(p0->indicator().functor() != p0->indicator().functor() ||
			   p0->indicator().arity()   != p0->indicator().arity()) {
				return false;
			}
			// unify all arguments
			for(uint32_t i=0; i<p0->indicator().arity(); ++i) {
				if(!unify(p0->arguments()[i], p1->arguments()[i])) {
					return false;
				}
			}
			break;
		}
		case TermType::STRING:
			return t1->type()==TermType::STRING &&
				((StringTerm*)t0.get())->value()==((StringTerm*)t1.get())->value();
		case TermType::DOUBLE:
			return t1->type()==TermType::DOUBLE &&
				((DoubleTerm*)t0.get())->value()==((DoubleTerm*)t1.get())->value();
		case TermType::INT32:
			return t1->type()==TermType::INT32 &&
				((Integer32Term*)t0.get())->value()==((Integer32Term*)t1.get())->value();
		case TermType::LONG:
			return t1->type()==TermType::LONG &&
				((LongTerm*)t0.get())->value()==((LongTerm*)t1.get())->value();
		default:
			KB_WARN("Ignoring unknown term type '{}'.", (int)t0->type());
			return false;
		}
	}
	
	return true;
}

bool Unifier::unify(const Variable &var, const TermPtr &t)
{
	// TODO: fail if var *occurs* in t (occurs check)
	//    - requirement: store set of variables with each term
	//      to avoid redundant computation, and to enable quick occurs test.
	set(var, t);
	return true;
}

TermPtr Unifier::apply()
{
	if(!exists_) {
		// no unifier exists
		return BottomTerm::get();
	}
	else if(mapping_.empty() ||
		t0_->isGround() ||
		t1_->type()==TermType::VARIABLE)
	{
		// empty unifier, or only substitutions in t1
		return t0_;
	}
	else if(t1_->isGround() ||
		t0_->type()==TermType::VARIABLE)
	{
		// only substitutions in t0
		return t1_;
	}
	else if(t0_->type()==TermType::PREDICATE) {
		// both t0_ and t1_ contain variables, so they are either Variables
		// or Predicates where an argument contains a variable.
		// the variable case if covered above so both must be predicates.
		// TODO: choose the one with less variables?
		Predicate *p = (Predicate*)t0_.get();
		return p->applySubstitution(*this);
	}
	else {
		KB_WARN("something went wrong.");
		return BottomTerm::get();
	}
}



namespace std {
	std::ostream& operator<<(std::ostream& os, const Term& t)
	{
		t.write(os);
		return os;
	}

	std::ostream& operator<<(std::ostream& os, const Substitution& omega)
	{
		uint32_t i=0;
		os << '{';
		for(const auto &pair : omega.mapping()) {
			os << pair.first.name() << ':' << ' ' << (*pair.second.get());
			if(++i < omega.mapping().size()) {
				os << ',';
			}
		}
		return os << '}';
	}
}

