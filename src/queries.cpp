/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <memory>
#include <sstream>
// SWI Prolog
#include <SWI-Prolog.h>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/queries.h>

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
	return std::all_of(formulae_.begin(), formulae_.end(),
				[](const std::shared_ptr<Formula> &x){ return x->isGround(); });
}

bool ConnectiveFormula::isGround() const
{
	return isGround_;
}

std::vector<std::shared_ptr<Formula>> ConnectiveFormula::applySubstitution1(
	const std::vector<std::shared_ptr<Formula>> &otherFormulas,
	const Substitution &sub)
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
	return std::make_shared<Query>(
		formula_->isGround() ?
		formula_ :
		formula_->applySubstitution(sub)
	);
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
		std::lock_guard<std::mutex> lock(channel_mutex_);
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
	static SubstitutionPtr x = std::make_shared<QueryResult>();
	return x;
}

SubstitutionPtr& QueryResultStream::eos()
{
	// TODO: make substitution immutable
	static SubstitutionPtr x = std::make_shared<QueryResult>();
	return x;
}

bool QueryResultStream::isOpened() const
{
	return isOpened_;
}

void QueryResultStream::push(const Channel &channel, const SubstitutionPtr &item)
{
	if(QueryResultStream::isEOS(item)) {
		// prevent channels from being created while processing EOS message
		std::lock_guard<std::mutex> lock(channel_mutex_);
		// remove channel once EOS is reached
		channels_.erase(channel.iterator_);
		
		// auto-close this stream if no channels are left,
		// also send EOS in this case.
		if(channels_.empty() && isOpened()) {
			isOpened_ = false;
			// TODO: lift lock before pushing
			push(item);
		}
	}
	else if(!isOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	}
	else {
		push(item);
	}
}


QueryResultStream::Channel::Channel(const std::shared_ptr<QueryResultStream> &stream)
: stream_(stream),
  isOpened_(true)
{
}

QueryResultStream::Channel::~Channel()
{
	close();
}

std::shared_ptr<QueryResultStream::Channel> QueryResultStream::Channel::create(const std::shared_ptr<QueryResultStream> &stream)
{
	std::lock_guard<std::mutex> lock(stream->channel_mutex_);
	if(stream->isOpened()) {
		auto channel = std::make_shared<QueryResultStream::Channel>(stream);
		stream->channels_.push_back(channel);
		channel->iterator_ = stream->channels_.end();
		--channel->iterator_;
		return channel;
	}
	else {
		throw QueryError("cannot create a channel of a closed stream");
	}
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
	else if(!QueryResultStream::isEOS(msg)) {
		KB_WARN("message pushed to closed stream {}", reinterpret_cast<std::uintptr_t>(this));
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
		pushToQueue(QueryResultStream::eos());
	}
}

void QueryResultQueue::pushToQueue(const SubstitutionPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);
		queue_.push(item);
	}
	queue_CV_.notify_one();
}

void QueryResultQueue::push(const SubstitutionPtr &item)
{
	pushToQueue(item);
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
		pushToBroadcast(QueryResultStream::eos());
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
	pushToBroadcast(item);
}

void QueryResultBroadcaster::pushToBroadcast(const SubstitutionPtr &item)
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
	// need to lock the whole push as genCombinations uses an iterator over the buffer.
	// TODO: support concurrent genCombinations call below
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	
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
		QueryResultBroadcaster::push(std::make_shared<Substitution>(*s));
	}
	else if(it->first == pushedChannelID) {
		// ignore pushed channel
	}
	else if(it->second.size()==1) {
		// only a single message buffered from this channel
		auto it1 = it; ++it1;
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
		auto it1 = it; ++it1;
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


QueryError::QueryError(const std::string& what)
: std::runtime_error(what)
{}

QueryError::QueryError(
	const Query &erroneousQuery,
	const Term &errorTerm)
: std::runtime_error(formatErrorString(erroneousQuery, errorTerm))
{
}

std::string QueryError::formatErrorString(
	const Query &erroneousQuery,
	const Term &errorTerm)
{
	std::stringstream buffer;
	buffer << errorTerm;
	return buffer.str();
}


namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Formula& phi)
	{
		phi.write(os);
		return os;
	}
	
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q)
	{
		os << *(q.formula().get());
		return os;
	}
}
