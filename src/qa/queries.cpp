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

void QueryResultStream::close()
{
	if(isOpened()) {
		for(auto &c : channels_) {
			c->isOpened_ = false;
		}
		channels_.clear();
		isOpened_ = false;
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

void QueryResultStream::push(const Channel &channel, const SubstitutionPtr &item)
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

void QueryResultStream::Channel::close()
{
	if(isOpened()) {
		stream_->push(*this, QueryResultStream::eos());
		isOpened_ = false;
	}
}

uint32_t QueryResultStream::Channel::id() const
{
	return reinterpret_cast<std::uintptr_t>(this);
}

void QueryResultStream::Channel::push(const QueryResultPtr &msg)
{
	if(isOpened()) {
		stream_->push(*this, msg);
		if(QueryResultStream::isEOS(msg)) {
			isOpened_ = false;
		}
	}
	else {
		spdlog::warn("message pushed to closed stream");
	}
}

bool QueryResultStream::Channel::isOpened() const
{
	return isOpened_;
}


QueryResultQueue::QueryResultQueue()
: QueryResultStream()
{}

QueryResultQueue::~QueryResultQueue()
{
	if(isOpened()) {
		push(QueryResultStream::eos());
	}
}

void QueryResultQueue::push(const SubstitutionPtr &item)
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
	SubstitutionPtr x = front();
	pop();
	return x;
}


QueryResultBroadcaster::QueryResultBroadcaster()
: QueryResultStream()
{}

QueryResultBroadcaster::~QueryResultBroadcaster()
{
	if(isOpened()) {
		push(QueryResultStream::eos());
	}
}

void QueryResultBroadcaster::addSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.push_back(subscriber);
}

void QueryResultBroadcaster::removeSubscriber(const std::shared_ptr<Channel> &subscriber)
{
	subscribers_.remove(subscriber);
}

void QueryResultBroadcaster::push(const SubstitutionPtr &item)
{
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}


QueryResultCombiner::QueryResultCombiner()
: QueryResultBroadcaster()
{}

void QueryResultCombiner::push(const Channel &channel, const SubstitutionPtr &msg)
{
	const uint32_t channelID = channel.id();
	
	// add to the buffer for later combinations
	buffer_[channelID].push_back(msg);
	
	// generate combinations with other channels if each channel
	// buffer has some content.
	if(buffer_.size() == channels_.size()) {
		SubstitutionPtr combination(new Substitution(*(msg.get())));
		// generate all combinations and push combined messages
		genCombinations(channelID, buffer_.begin(), combination);
	}
}

void QueryResultCombiner::genCombinations(
		uint32_t pushedChannelID,
		QueryResultBuffer::iterator it,
		SubstitutionPtr &combinedSubstitution)
{
	if(it == buffer_.end()) {
		// end reached, push combination
		// note: need to create a new SubstitutionPtr due to the rollBack below.
		// TODO: it could be that the same combination is generated
		// multiple time, avoid redundant push here?
		Substitution *s = combinedSubstitution.get();
		QueryResultBroadcaster::push(SubstitutionPtr(new Substitution(*s)));
	}
	else if(it->first == pushedChannelID) {
		// ignore pushed channel
	}
	else if(it->second.size()==1) {
		// only a single message buffered from this channel
		QueryResultBuffer::iterator it1 = it; ++it1;
		// keep track of changes for rolling them back
		Substitution::Diff changes;
		// combine and continue with next channel.
		// this is done to avoid creating many copies of the substitution map.
		if(combinedSubstitution->combine(it->second.front(), changes)) {
			genCombinations(pushedChannelID, it1, combinedSubstitution);
		}
		// roll back changes
		combinedSubstitution->rollBack(changes);
	}
	else {
		// generate a combination for each buffered message
		// note: the number of possible combinations grows
		// exponentially with number of messages in channels
		// TODO: it might be good to think about ways of discarding options quicker.
		// - one way would be to cache all possible combinations, but could be many of them.
		//   then only these would need to be iterated here, but additionally pre-computing
		//   must happen here
		QueryResultBuffer::iterator it1 = it; ++it1;
		// keep track of changes for rolling them back
		Substitution::Diff changes;
		// combine each buffered message
		for(auto &msg : it->second) {
			// combine and continue with next channel
			if(combinedSubstitution->combine(msg, changes)) {
				genCombinations(pushedChannelID, it1, combinedSubstitution);
			}
			// roll back changes, note this also clears the `changes` object
			// so it can be re-used in the next iteration
			combinedSubstitution->rollBack(changes);
		}
	}
}

