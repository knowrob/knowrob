/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <sstream>
// logging
#include <spdlog/spdlog.h>
// SWI Prolog
#include <SWI-Prolog.h>
// KnowRob
#include <knowrob/qa/queries.h>

using namespace knowrob;

/******************************************/
/*************** Formulas *****************/
/******************************************/

Formula::Formula(const FormulaType &type)
: type_(type)
{}

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

std::ostream& operator<<(std::ostream& os, const Formula& phi)
{
	phi.write(os);
	return os;
}


ConnectiveFormula::ConnectiveFormula(FormulaType type,
	const std::vector<std::shared_ptr<Formula>> &formulae)
: Formula(type),
  formulae_(formulae),
  isGround_(isGround1())
{
}

ConnectiveFormula::ConnectiveFormula(const ConnectiveFormula &other, const Substitution &sub)
: Formula(other.type_),
  formulae_(applySubstitution1(other.formulae_, sub)),
  isGround_(isGround1())
{
}

bool ConnectiveFormula::isGround1() const
{
	for(const auto &x : formulae_) {
		if(!x->isGround()) return false;
	}
	return true;
}

bool ConnectiveFormula::isGround() const
{
	return isGround_;
}

std::vector<std::shared_ptr<Formula>> ConnectiveFormula::applySubstitution1(
	const std::vector<std::shared_ptr<Formula>> &otherFormulas,
	const Substitution &sub) const
{
	std::vector<std::shared_ptr<Formula>> out(otherFormulas.size());
	
	for(uint32_t i=0; i<otherFormulas.size(); i++) {
		out[i] = (otherFormulas[i]->isGround() ?
			otherFormulas[i] :
			otherFormulas[i]->applySubstitution(sub));
	}
	
	return out;
}

void ConnectiveFormula::write(std::ostream& os) const
{
	os << '(';
	for(uint32_t i=0; i<formulae_.size(); i++) {
		os << *(formulae_[i].get());
		if(i+1 < formulae_.size()) {
			os << ' ' << operator_symbol() << ' ';
		}
	}
	os << ')';
}


ConjunctionFormula::ConjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
: ConnectiveFormula(FormulaType::CONJUNCTION, formulae)
{
}

ConjunctionFormula::ConjunctionFormula(const ConjunctionFormula &other, const Substitution &sub)
: ConnectiveFormula(other, sub)
{
}

std::shared_ptr<Formula> ConjunctionFormula::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<ConjunctionFormula>(
		new ConjunctionFormula(*this, sub));
}

DisjunctionFormula::DisjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
: ConnectiveFormula(FormulaType::DISJUNCTION, formulae)
{
}

DisjunctionFormula::DisjunctionFormula(const DisjunctionFormula &other, const Substitution &sub)
: ConnectiveFormula(other, sub)
{
}

std::shared_ptr<Formula> DisjunctionFormula::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<DisjunctionFormula>(
		new DisjunctionFormula(*this, sub));
}

PredicateFormula::PredicateFormula(const std::shared_ptr<Predicate> &predicate)
: Formula(FormulaType::PREDICATE),
  predicate_(predicate)
{
}

bool PredicateFormula::isGround() const
{
	return predicate_->isGround();
}

std::shared_ptr<Formula> PredicateFormula::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Formula>(new PredicateFormula(
		predicate_->applySubstitution(sub)));
}

void PredicateFormula::write(std::ostream& os) const
{
	predicate_->write(os);
}

/******************************************/
/**************** Queries *****************/
/******************************************/

Query::Query(const std::shared_ptr<Formula> &formula)
: formula_(formula)
{
}

Query::Query(const std::shared_ptr<Predicate> &predicate)
: formula_(new PredicateFormula(predicate))
{
}

std::shared_ptr<Query> Query::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Query>(new Query(
		formula_->isGround() ?
		formula_ :
		formula_->applySubstitution(sub)
	));
}

std::ostream& operator<<(std::ostream& os, const Query& q)
{
	os << *(q.formula().get());
	return os;
}

std::string Query::getHumanReadableString() const
{
	std::ostringstream ss;
	ss << *this;
	return ss.str();
}


/******************************************/
/************* Query results **************/
/******************************************/

QueryResultStream::QueryResultStream()
: isOpened_(true)
{}

QueryResultStream::~QueryResultStream()
{
	close();
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

std::shared_ptr<QueryResultStream::Channel> QueryResultStream::createChannel()
{
	auto channel = std::shared_ptr<QueryResultStream::Channel>(
		new QueryResultStream::Channel(this));
	channels_.push_back(channel);
	channel->iterator_ = channels_.end();
	--channel->iterator_;
	return channel;
}

bool QueryResultStream::isOpened() const
{
	return isOpened_;
}

void QueryResultStream::close()
{
	if(isOpened()) {
		push(QueryResultStream::eos());
		channels_.clear();
		isOpened_ = false;
	}
}

void QueryResultStream::push(const Channel &channel, SubstitutionPtr &item)
{
	if(QueryResultStream::isEOS(item)) {
		// remove channel once EOS is reached
		channels_.erase(channel.iterator_);
		
		// auto-close this stream if no channels are left,
		// also send EOF in this case.
		if(channels_.empty() && isOpened()) {
			isOpened_ = false;
			push(item);
		}
	}
	else if(!isOpened()) {
		spdlog::warn("ignoring attempt to write to a closed stream.");
	}
	else {
		push(item);
	}
}


QueryResultStream::Channel::Channel(QueryResultStream *stream)
: stream_(stream),
  isOpened_(true)
{
}

QueryResultStream::Channel::~Channel()
{
	close();
}

uint32_t QueryResultStream::Channel::id() const
{
	return reinterpret_cast<std::uintptr_t>(this);
}

void QueryResultStream::Channel::push(QueryResultPtr &msg)
{
	stream_->push(*this, msg);
	if(QueryResultStream::isEOS(msg)) {
		isOpened_ = false;
	}
}

bool QueryResultStream::Channel::isOpened() const
{
	return isOpened_;
}

void QueryResultStream::Channel::close()
{
	if(isOpened()) {
		stream_->push(*this, QueryResultStream::eos());
		isOpened_ = false;
	}
}


QueryResultQueue::QueryResultQueue()
: QueryResultStream()
{}

void QueryResultQueue::push(SubstitutionPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);
		queue_.push(item);
	}
	queue_CV_.notify_one();
}
		
SubstitutionPtr& QueryResultQueue::front()
{
	std::unique_lock<std::mutex> lock(queue_mutex_);
	queue_CV_.wait(lock, [&]{ return !queue_.empty(); });
	return queue_.front();
}

void QueryResultQueue::pop()
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_.pop();
}

SubstitutionPtr QueryResultQueue::pop_front()
{
	SubstitutionPtr & x = front();
	pop();
	return x;
}


QueryResultBroadcaster::QueryResultBroadcaster()
: QueryResultStream()
{}

void QueryResultBroadcaster::addSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.push_back(subscriber);
}

void QueryResultBroadcaster::removeSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.remove(subscriber);
}

void QueryResultBroadcaster::push(SubstitutionPtr &item)
{
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}


QueryResultCombiner::QueryResultCombiner()
: QueryResultBroadcaster()
{}

void QueryResultCombiner::push(const Channel &channel, SubstitutionPtr &msg)
{
	const uint32_t channelID = channel.id();
	
	// add to the buffer for later combinations
	buffer_[channelID].push_back(msg);
	
	// generate combinations with other channels if each channel
	// buffer has some content.
	if(buffer_.size() == channels_.size()) {
		// TODO could use std::stack instead, is it faster?
		std::list<SubstitutionPtr> combination;
		// add msg for channelID
		combination.push_back(msg);
		// generate all combinations and push combined msg
		genCombinations(channelID, buffer_.begin(), combination);
	}
}

void QueryResultCombiner::genCombinations(
		uint32_t pushedChannelID,
		QueryResultBuffer::iterator it,
		std::list<SubstitutionPtr> &combination)
{
	if(it == buffer_.end()) {
		// end reached push combination if possible
		// TODO: ugly interface!
		SubstitutionPtr combined;
		//SubstitutionPtr combined = Substitution::combine(combination);
		if(combined.get()!=NULL) {
			QueryResultBroadcaster::push(combined);
		}
	}
	else if(it->first == pushedChannelID) {
		// ignore pushed channel
	}
	else if(it->second.size()==1) {
		// only a single message buffered from this channel
		combination.push_back(it->second.front());
	}
	else {
		// generate a combination for each buffered message
		std::list<SubstitutionPtr>::iterator jt;
		QueryResultBuffer::iterator it1 = it;
		++it1;
		
		// TODO: the number of possible combinations grows
		// exponentially with number of messages in channels so
		// it might be good to think about ways of discarding options quicker.
		// - one way would be to cache all possible combinations, but could be many of them.
		//   then only these would need to be iterated here, but additionally pre-computing
		//   must happen here
		// - maybe it would be better to pairwise unification in the loop for early breaking
		//   out of it and avoiding redundancies.
		for(auto &msg : it->second) {
			// push msg and remember position
			combination.push_back(msg);
			jt = combination.end();
			--jt;
			// process other channels with `msg`
			genCombinations(pushedChannelID, it1, combination);
			// remove again
			combination.erase(jt);
		}
	}
}
