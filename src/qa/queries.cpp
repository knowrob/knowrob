/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// logging
#include <spdlog/spdlog.h>
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

ConnectiveFormula::ConnectiveFormula(const FormulaType &type,
	const std::vector<std::shared_ptr<Formula>> &formulae)
: Formula(type),
  formulae_(formulae)
{
}

bool ConnectiveFormula::hasFreeVariable() const
{
	for(auto const x : formulae_) {
		if(x->hasFreeVariable()) return true;
	}
	return false;
}

void ConnectiveFormula::applySubstitution(const Substitution &sub)
{
	for(auto const x : formulae_) {
		x->applySubstitution(sub);
	}
}

ConjunctionFormula::ConjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
: ConnectiveFormula(FormulaType::CONJUNCTION, formulae)
{
}

DisjunctionFormula::DisjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
: ConnectiveFormula(FormulaType::DISJUNCTION, formulae)
{
}

PredicateFormula::PredicateFormula(const std::shared_ptr<Predicate> &predicate)
: Formula(FormulaType::PREDICATE),
  predicate_(predicate)
{
}

bool PredicateFormula::hasFreeVariable() const
{
	return predicate_->hasFreeVariable();
}

void PredicateFormula::applySubstitution(const Substitution &sub)
{
	if(predicate_->hasFreeVariable() && predicate_->type() == TermType::PREDICATE) {
		predicate_->applySubstitution(sub);
	}
}


/******************************************/
/************* query objects **************/
/******************************************/

Query::Query(const std::shared_ptr<Formula> &formula)
: formula_(formula)
{
}

Query::Query(const std::shared_ptr<Predicate> &predicate)
: formula_(new PredicateFormula(predicate))
{
}

Query::Query(const Query &other)
: formula_(copyFormula(other.formula_))
{
}

std::shared_ptr<Formula> Query::copyFormula(const std::shared_ptr<Formula> &phi)
{
	if(!phi->hasFreeVariable()) {
		// formulae without free variables are read-only so
		// we can safely re-use the reference here.
		return phi;
	}
	
	switch(phi->type()) {
	case FormulaType::PREDICATE: {
		std::shared_ptr<Term> t = copyTerm(((PredicateFormula*)phi.get())->predicate());
		return std::shared_ptr<Formula>(new PredicateFormula(
			std::static_pointer_cast<Predicate>(t)
		));
	}
	case FormulaType::DISJUNCTION:
	case FormulaType::CONJUNCTION: {
		ConnectiveFormula *phi0 = (ConnectiveFormula*)phi.get();
		// copy arguments
		std::vector<std::shared_ptr<Formula>> arguments(phi0->formulae().size());
		for(uint32_t i=0; i<phi0->formulae().size(); i++) {
			arguments[i] = copyFormula(phi0->formulae()[i]);
		}
		if(phi->type()==FormulaType::DISJUNCTION)
			return std::shared_ptr<Formula>(new DisjunctionFormula(arguments));
		if(phi->type()==FormulaType::CONJUNCTION)
			return std::shared_ptr<Formula>(new ConjunctionFormula(arguments));
		break;
	}
	default:
		spdlog::warn("Ignoring unknown formula type '{}' in query.", phi->type());
		break;
	}
		
	return phi;
}

std::shared_ptr<Term> Query::copyTerm(const std::shared_ptr<Term> &t)
{
	switch(t->type()) {
	case TermType::PREDICATE: {
		Predicate *p = (Predicate*)t.get();
		if(p->hasFreeVariable()) {
			// copy arguments
			uint32_t num_args = p->arguments().size();
			std::vector<std::shared_ptr<Term>> arguments(num_args);
			for(uint32_t i=0; i<num_args; i++) {
				arguments[i] = copyTerm(p->arguments()[i]);
			}
			// create a new predicate object
			// TODO: copy of functor string is not needed.
			return std::shared_ptr<Term>(new Predicate(p->indicator().functor(), arguments));
		}
		else {
			return t;
		}
	}
	// handle immutable terms
	case TermType::VARIABLE:
	case TermType::STRING:
	case TermType::DOUBLE:
	case TermType::INT32:
	case TermType::LONG:
		return t;
	default:
		spdlog::warn("Ignoring unknown term type '{}'.", t->type());
		return t;
	}
}

void Query::applySubstitution(const Substitution &sub)
{
	formula_->applySubstitution(sub);
}

std::string Query::toString() const
{
	// TODO
	return "";
}


/******************************************/
/************* query results **************/
/******************************************/

QueryResultStream::QueryResultStream()
{}

QueryResultStream::~QueryResultStream()
{
	for(auto &x : subscriptions_) {
		x->removeSubscriber(this);
	}
}

bool QueryResultStream::isEOS(const SubstitutionPtr &item)
{
	return (item.get() == eos().get());
}

SubstitutionPtr& QueryResultStream::bos()
{
	// TODO: make substitution immutable
	static SubstitutionPtr x = SubstitutionPtr(new QueryResult);
	return x;
}

SubstitutionPtr& QueryResultStream::eos()
{
	// TODO: make substitution immutable
	static SubstitutionPtr x = SubstitutionPtr(new QueryResult);
	return x;
}

QueryResultBroadcast::QueryResultBroadcast()
: QueryResultStream(),
  numEOSPushed_(0)
{
}

QueryResultBroadcast::~QueryResultBroadcast()
{
	for(auto &x : subscribers_) {
		removeSubscriber(x);
	}
}

void QueryResultBroadcast::addSubscriber(QueryResultStream *subscriber)
{
	subscribers_.push_back(subscriber);
	subscriber->subscriptions_.push_back(this);
}

void QueryResultBroadcast::removeSubscriber(QueryResultStream *subscriber)
{
	subscribers_.remove(subscriber);
	subscriber->subscriptions_.remove(this);
}

void QueryResultBroadcast::push(SubstitutionPtr &item)
{
	if(QueryResultStream::isEOS(item)) {
		// FIXME: broadcast EOS may never be published if a subscription channel is removed
		//        after EOS has been sent via this channel.
		numEOSPushed_ += 1;
		if(numEOSPushed_ < subscriptions_.size()) {
			return;
		}
	}
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}


QueryResultQueue::QueryResultQueue()
: QueryResultStream()
{}

void QueryResultQueue::push(SubstitutionPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		queue_.push(item);
	}
	cond_var_.notify_all();
}
		
SubstitutionPtr& QueryResultQueue::front()
{
	std::unique_lock<std::mutex> lock(mutex_);
	// wait until SubstitutionPtr at index is available
	cond_var_.wait(lock, [&]{ return !queue_.empty(); });
	return queue_.front();
}

void QueryResultQueue::pop()
{
	std::lock_guard<std::mutex> lock(mutex_);
	queue_.pop();
}

SubstitutionPtr QueryResultQueue::pop_front()
{
	SubstitutionPtr & x = front();
	pop();
	return x;
}

