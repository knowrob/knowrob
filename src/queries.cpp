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
#include <limits>
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
/************ Scoped queries **************/
/******************************************/

TimePoint::TimePoint(const double& value)
: value_(value)
{
}

TimePoint TimePoint::now()
{
	auto time = std::chrono::system_clock::now().time_since_epoch();
	std::chrono::seconds seconds = std::chrono::duration_cast< std::chrono::seconds >(time);
	std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(time);
	return { (double) seconds.count() + ((double) (ms.count() % 1000)/1000.0) };
}

bool TimePoint::operator<(const TimePoint& other) const
{
	return value_ < other.value_;
}


TimeInterval::TimeInterval(const Range<TimePoint> &sinceRange, const Range<TimePoint> &untilRange)
: FuzzyInterval<TimePoint>(sinceRange, untilRange)
{}

const TimeInterval& TimeInterval::anytime()
{
	// sinceRange: [*,*], untilRange: [*,*]
	static const TimeInterval timeInterval(
			Range<TimePoint>(std::nullopt, std::nullopt),
			Range<TimePoint>(std::nullopt, std::nullopt));
	return timeInterval;
}

TimeInterval TimeInterval::currently()
{
	// sinceRange: [*,Now], untilRange: [Now,*]
	TimePoint now = TimePoint::now();
	return {Range<TimePoint>(std::nullopt, now),
			Range<TimePoint>(now, std::nullopt) };
}

std::shared_ptr<TimeInterval> TimeInterval::intersectWith(const TimeInterval &other) const
{
	return std::make_shared<TimeInterval>(
			minRange_.intersectWith(other.minRange_),
			maxRange_.intersectWith(other.maxRange_));
}

ConfidenceValue::ConfidenceValue(double value)
: value_(value)
{
}

const ConfidenceValue& ConfidenceValue::max()
{
	static ConfidenceValue v(0.0);
	return v;
}

const ConfidenceValue& ConfidenceValue::min()
{
	static ConfidenceValue v(1.0);
	return v;
}

bool ConfidenceValue::operator<(const ConfidenceValue& other) const
{
	return value_ < other.value_;
}

bool ConfidenceValue::operator==(const ConfidenceValue& other) const
{
	return this == &other || fabs(value_ - other.value_) < std::numeric_limits<double>::epsilon();
}

ConfidenceInterval::ConfidenceInterval(const Range<ConfidenceValue> &minRange,
									   const Range<ConfidenceValue> &maxRange)
: FuzzyInterval<ConfidenceValue>(minRange,maxRange)
{
}

const ConfidenceInterval& ConfidenceInterval::any()
{
	// minRange: [*,*], maxRange: [*,*]
	static const ConfidenceInterval interval(
			Range<ConfidenceValue>(std::nullopt, std::nullopt),
			Range<ConfidenceValue>(std::nullopt, std::nullopt));
	return interval;
}

const ConfidenceInterval& ConfidenceInterval::certain()
{
	// minRange: [MAX_CONFIDENCE,MAX_CONFIDENCE], maxRange: [MAX_CONFIDENCE,MAX_CONFIDENCE]
	static const ConfidenceInterval interval(
			Range<ConfidenceValue>(ConfidenceValue::max(), ConfidenceValue::max()),
			Range<ConfidenceValue>(ConfidenceValue::max(), ConfidenceValue::max()));
	return interval;
}

ConfidenceInterval ConfidenceInterval::at_least(const ConfidenceValue &value)
{
	// minRange: [$VALUE,*], maxRange: [*,*]
	return { Range<ConfidenceValue>(value, std::nullopt),
			 Range<ConfidenceValue>(std::nullopt, std::nullopt) };
}


QueryResult::QueryResult()
: substitution_(std::make_shared<Substitution>()),
  o_timeInterval_(std::nullopt),
  o_confidence_(std::nullopt)
{
}

QueryResult::QueryResult(const std::shared_ptr<Substitution> &substitution)
: substitution_(substitution),
  o_timeInterval_(std::nullopt),
  o_confidence_(std::nullopt)
{
}

QueryResult::QueryResult(const QueryResult &other)
: substitution_(std::make_shared<Substitution>(*other.substitution_))
{
	setTimeInterval(other.timeInterval_);
	setConfidenceValue(other.confidence_);
}

const std::shared_ptr<const QueryResult>& QueryResult::emptyResult()
{
	static auto result = std::make_shared<const QueryResult>();
	return result;
}

void QueryResult::substitute(const Variable &var, const TermPtr &term)
{
	substitution_->set(var, term);
}

bool QueryResult::hasSubstitution(const Variable &var)
{
	return substitution_->contains(var);
}

void QueryResult::setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval)
{
	timeInterval_ = timeInterval;
	o_timeInterval_ = (timeInterval_ ?
			std::optional<const TimeInterval*>(timeInterval_.get()) :
			std::optional<const TimeInterval*>(std::nullopt));
}

void QueryResult::setConfidenceValue(const std::shared_ptr<ConfidenceValue> &confidence)
{
	confidence_ = confidence;
	o_confidence_ = (confidence_ ?
			std::optional<const ConfidenceValue*>(confidence_.get()) :
			std::optional<const ConfidenceValue*>(std::nullopt));
}

bool QueryResult::combine(std::shared_ptr<QueryResult> &other, Reversible *changes)
{
	// unify substitutions
	if(!substitution_->unifyWith(other->substitution_, changes)) {
		// unification failed -> results cannot be combined
		return false;
	}
	// compute intersection of time intervals
	if(combineTimeInterval(other->timeInterval_, changes)) {
		// intersection empty -> results cannot be combined
		if(timeInterval_->empty()) return false;
	}
	// accumulate confidence values
	auto oldConfidence = confidence_;
	if(combineConfidence(other->confidence_)) {
		if(changes) changes->push([this,oldConfidence](){
			setConfidenceValue(oldConfidence);
		});
	}
	return true;
}

bool QueryResult::combineTimeInterval(const std::shared_ptr<TimeInterval> &otherTimeInterval, Reversible *reversible)
{
	if(otherTimeInterval) {
		if(o_timeInterval_.has_value()) {
			if(!otherTimeInterval->isMoreGeneralThan(*timeInterval_)) {
				auto old = timeInterval_;
				setTimeInterval(timeInterval_->intersectWith(*otherTimeInterval));
				if(reversible) reversible->push([this,old](){ setTimeInterval(old); });
				return true;
			}
		}
		else {
			setTimeInterval(otherTimeInterval);
			if(reversible) reversible->push([this](){ setTimeInterval(std::shared_ptr<TimeInterval>()); });
			return true;
		}
	}
	return false;
}

bool QueryResult::combineConfidence(const std::shared_ptr<ConfidenceValue> &otherConfidence)
{
	if(otherConfidence) {
		if(confidence_) {
			auto newConfidence = otherConfidence->value() * confidence_->value();
			setConfidenceValue(std::make_shared<ConfidenceValue>(newConfidence));
			return true;
		}
		else {
			setConfidenceValue(otherConfidence);
			return true;
		}
	}
	else {
		return false;
	}
}

/******************************************/
/**************** Queries *****************/
/******************************************/

Query::Query(const std::shared_ptr<Formula> &formula)
: formula_(formula),
  o_timeInterval_(std::nullopt),
  o_confidenceInterval_(std::nullopt)
{
}

Query::Query(const std::shared_ptr<Predicate> &predicate)
: formula_(new PredicateFormula(predicate)),
  o_timeInterval_(std::nullopt),
  o_confidenceInterval_(std::nullopt)
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

void Query::setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval)
{
	timeInterval_ = timeInterval;
	o_timeInterval_ = (timeInterval_ ?
					   std::optional<const TimeInterval*>(timeInterval_.get()) :
					   std::optional<const TimeInterval*>(std::nullopt));
}

void Query::setConfidenceInterval(const std::shared_ptr<ConfidenceInterval> &confidenceInterval)
{
	confidenceInterval_ = confidenceInterval;
	o_confidenceInterval_ = (confidenceInterval_ ?
					 std::optional<const ConfidenceInterval*>(confidenceInterval_.get()) :
					 std::optional<const ConfidenceInterval*>(std::nullopt));
}


QueryInstance::QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
							 const std::shared_ptr<QueryResultStream::Channel> &outputChannel,
							 const std::shared_ptr<const QueryResult> &partialResult)
: uninstantiatedQuery_(uninstantiatedQuery),
  outputChannel_(outputChannel),
  partialResult_(partialResult)
{
}

QueryInstance::QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
							 const std::shared_ptr<QueryResultStream::Channel> &outputChannel)
: uninstantiatedQuery_(uninstantiatedQuery),
  outputChannel_(outputChannel),
  partialResult_(QueryResult::emptyResult())
{
}

std::shared_ptr<const Query> QueryInstance::create()
{
	// TODO: set scope for query generated, else would be dangerous because
	//   both QueryInstance and Query have an interface for getting the scope.
	if(partialResult_->substitution()->empty()) {
		return uninstantiatedQuery_;
	}
	else {
		return uninstantiatedQuery_->applySubstitution(*partialResult_->substitution());
	}
}

void QueryInstance::pushSolution(const std::shared_ptr<QueryResult> &solution)
{
	// include substitutions of partialResult_
	for (const auto& pair: *partialResult_->substitution())
		if (!solution->hasSubstitution(pair.first)) {
			solution->substitute(pair.first, pair.second);
		} else {
			// assume both terms unify
		}

	// combine time intervals
	std::shared_ptr<TimeInterval> otherTimeInterval = (
			partialResult_->timeInterval().has_value() ?
			partialResult_->timeInterval_ :
			uninstantiatedQuery_->timeInterval_);
	solution->combineTimeInterval(otherTimeInterval);
	// ignore solution if time interval is empty
	if(solution->timeInterval_ && solution->timeInterval_->empty()) return;

	// combine confidence values
	solution->combineConfidence(partialResult_->confidence_);
	// ignore solution if confidence value does not lie within desired interval
	if(uninstantiatedQuery_->confidenceInterval_ && solution->confidence_ &&
	  !uninstantiatedQuery_->confidenceInterval_->contains(*solution->confidence_)) {
		return;
	}

	// finally push result into the stream
	outputChannel_->push(solution);
}

void QueryInstance::pushEOS()
{
	outputChannel_->push(QueryResultStream::eos());
}

const std::optional<const TimeInterval*>& QueryInstance::timeInterval() const
{
	if(partialResult_->timeInterval().has_value()) {
		return partialResult_->timeInterval();
	}
	else {
		return uninstantiatedQuery_->timeInterval();
	}
}

const std::optional<const ConfidenceInterval*>& QueryInstance::confidenceInterval() const
{
	return uninstantiatedQuery_->confidenceInterval();
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

bool QueryResultStream::isEOS(const QueryResultPtr &item)
{
	return (item.get() == eos().get());
}

QueryResultPtr& QueryResultStream::bos()
{
	// TODO: make immutable
	static auto msg = std::make_shared<QueryResult>();
	return msg;
}

QueryResultPtr& QueryResultStream::eos()
{
	// TODO: make immutable
	static auto msg = std::make_shared<QueryResult>();
	return msg;
}

bool QueryResultStream::isOpened() const
{
	return isOpened_;
}

void QueryResultStream::push(const Channel &channel, const QueryResultPtr &item)
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

void QueryResultQueue::pushToQueue(const QueryResultPtr &item)
{
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);
		queue_.push(item);
	}
	queue_CV_.notify_one();
}

void QueryResultQueue::push(const QueryResultPtr &item)
{
	pushToQueue(item);
}

QueryResultPtr& QueryResultQueue::front()
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

QueryResultPtr QueryResultQueue::pop_front()
{
	QueryResultPtr x = front();
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

void QueryResultBroadcaster::push(const QueryResultPtr &item)
{
	pushToBroadcast(item);
}

void QueryResultBroadcaster::pushToBroadcast(const QueryResultPtr &item)
{
	// broadcast the query result to all subscribers
	for(auto &x : subscribers_) {
		x->push(item);
	}
}


QueryResultCombiner::QueryResultCombiner()
: QueryResultBroadcaster()
{}

void QueryResultCombiner::push(const Channel &channel, const QueryResultPtr &msg)
{
	const uint32_t channelID = channel.id();
	// need to lock the whole push as genCombinations uses an iterator over the buffer.
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	
	// add to the buffer for later combinations
	buffer_[channelID].push_back(msg);
	
	// generate combinations with other channels if each channel
	// buffer has some content.
	if(buffer_.size() == channels_.size()) {
		QueryResultPtr combination(new QueryResult(*(msg.get())));
		// generate all combinations and push combined messages
		genCombinations(channelID, buffer_.begin(), combination);
	}
}

void QueryResultCombiner::genCombinations(
		uint32_t pushedChannelID,
		QueryResultBuffer::iterator it,
		QueryResultPtr &combinedResult)
{
	if(it == buffer_.end()) {
		// end reached, push combination
		// note: need to create a new SubstitutionPtr due to the rollBack below.
		// TODO: it could be that the same combination is generated multiple times, avoid redundant push here?
		QueryResultBroadcaster::push(std::make_shared<QueryResult>(*combinedResult));
	}
	else if(it->first == pushedChannelID) {
		// ignore pushed channel
	}
	else if(it->second.size()==1) {
		// only a single message buffered from this channel
		auto it1 = it; ++it1;
		// keep track of changes for rolling them back
		Reversible changes;
		// combine and continue with next channel.
		// this is done to avoid creating many copies of the substitution map.
		if(combinedResult->combine(it->second.front(), &changes)) {
			genCombinations(pushedChannelID, it1, combinedResult);
		}
		// roll back changes
		changes.rollBack();
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
		Reversible changes;
		// combine each buffered message
		for(auto &msg : it->second) {
			// combine and continue with next channel
			if(combinedResult->combine(msg, &changes)) {
				genCombinations(pushedChannelID, it1, combinedResult);
			}
			// roll back changes, note this also clears the `changes` object
			// so it can be re-used in the next iteration
			changes.rollBack();
		}
	}
}

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