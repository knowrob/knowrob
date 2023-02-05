/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERIES_H_
#define KNOWROB_QUERIES_H_

// STD
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <string>
#include <atomic>
#include <optional>
// KnowRob
#include <knowrob/ThreadPool.h>
#include <knowrob/terms.h>
#include <knowrob/logging.h>

namespace knowrob {
	/**
	 * The type of a formula.
	 */
	enum class FormulaType {
		// A formula of the form `P(t_1,..,t_n)` where each ti is a term
		// and "P" is a n-ary predicate symbol (or functor).
		PREDICATE,
		// A formula of the form `phi_1 AND ... AND phi_n` where each phi_i is a formula.
		CONJUNCTION,
		// A formula of the form `phi_1 OR ... OR phi_n` where each phi_i is a formula.
		DISJUNCTION
		// TODO handle more types of formulae
		// EQUALITY / UNIFICATION
		// IMPLICATION
		// NEGATION
		// ONCE / IGNORE
		// FORALL
	};
	
	/**
	 * A propositional formula.
	 * Note that all formulas are immutable.
	 */
	class Formula {
	public:
		/**
		 * @type the type of the formula.
		 */
		explicit Formula(const FormulaType &type);
		
		/**
		 * @return the type of this formula.
		 */
		FormulaType type() const { return type_; }
		
		/**
		 * Is this formula free of subformulas?
		 *
		 * @return true if this formula is atomic.
		 */
		bool isAtomic() const;
		
		/**
		 * @return true if this formula isMoreGeneralThan no free variables.
		 */
		virtual bool isGround() const = 0;
		
		/**
		 * Replaces variables in the formula with terms.
		 * @sub a substitution mapping.
		 * @return the created formula.
		 */
		virtual std::shared_ptr<Formula> applySubstitution(const Substitution &sub) const = 0;
		
		/**
		 * Write the formula into an ostream.
		 */
		virtual void write(std::ostream& os) const = 0;
	
	protected:
		const FormulaType type_;
	};
	
	// alias declaration
	using FormulaPtr = std::shared_ptr<Formula>;
	
	/**
	 * A predicate formula.
	 */
	class PredicateFormula : public Formula {
	public:
		/**
		 * @predicate a predicate reference.
		 */
		explicit PredicateFormula(const std::shared_ptr<Predicate> &predicate);
		
		/**
		 * @return the predicate associated to this formula.
		 */
		const std::shared_ptr<Predicate>& predicate() const { return predicate_; }
		
		// Override Formula
		bool isGround() const override;
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override Formula
		void write(std::ostream& os) const override;
		
	protected:
		const std::shared_ptr<Predicate> predicate_;
	};
	
	/**
	 * A formula with sub-formulas linked via logical connectives.
	 */
	class ConnectiveFormula : public Formula {
	public:
		/**
		 * @type the type of the formula.
		 * @formulae list of connected formulae.
		 */
		ConnectiveFormula(FormulaType type, const std::vector<FormulaPtr> &formulae);
		
		/**
		 * @return the sub-formulas associated to this formula.
		 */
		const std::vector<FormulaPtr>& formulae() const { return formulae_; }
		
		/**
		 * @return symbol string of the operator
		 */
		virtual const char* operator_symbol() const = 0;
		
		// Override Formula
		bool isGround() const override;
		
		// Override Formula
		void write(std::ostream& os) const override;
	
	protected:
		const std::vector<FormulaPtr> formulae_;
		const bool isGround_;
		
		ConnectiveFormula(const ConnectiveFormula &other, const Substitution &sub);
		
		bool isGround1() const;
		
		static std::vector<FormulaPtr> applySubstitution1(
			const std::vector<FormulaPtr> &otherFormulas,
			const Substitution &sub);
	};
	
	/**
	 * A conjunctive expression.
	 */
	class ConjunctionFormula : public ConnectiveFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		explicit ConjunctionFormula(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const override { return "\u2227"; }
		
	protected:
		ConjunctionFormula(const ConjunctionFormula &other, const Substitution &sub);
	};
	
	/**
	 * A disjunctive expression.
	 */
	class DisjunctionFormula : public ConnectiveFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		explicit DisjunctionFormula(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const override;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const override { return "\u2228"; }
	
	protected:
		DisjunctionFormula(const DisjunctionFormula &other, const Substitution &sub);
	};

	/**
	 * A value range [min,max]
	 * @tparam ValueType the type of values
	 */
	template <class ValueType> class Range {
	public:
		/**
		 * @param min the minimum value of the range
		 * @param max the maximum value of the range
		 */
		Range(const std::optional<ValueType> &min, const std::optional<ValueType> &max)
		: min_(min), max_(max) {}

		/**
		 * @return the minimum value of the range
		 */
		const std::optional<ValueType>& min() const { return min_; }

		/**
		 * @return the maximum value of the range
		 */
		const std::optional<ValueType>& max() const { return max_; }

		/**
		 * @return true if the range has no elements, i.e. max<min
		 */
		bool empty() const { return max_.has_value() && min_.has_value() && max_.value() < min_.value(); }

		/**
		 * @param other another range.
		 * @return true if this range isMoreGeneralThan another one.
		 */
		bool contains(const Range<ValueType> &other) const {
			return !((min_.has_value() && (!other.min_.has_value() || other.min_.value() < min_.value())) ||
					 (max_.has_value() && (!other.max_.has_value() || max_.value() < other.max_.value())));
		}

		/**
		 * Intersect this range with another one.
		 * @param other another range.
		 */
		Range<ValueType> intersectWith(const Range<ValueType> &other) const {
			auto *min = &min_;
			auto *max = &max_;
			if(other.min_.has_value() && (!min_.has_value() ||
				    min_.value() < other.min_.value())) {
				min = &other.min_;
			}
			if(other.max_.has_value() && (!max_.has_value() ||
					other.max_.value() < max_.value())) {
				max = &other.max_;
			}
			return { *min, *max };
		}

		bool hasValue() const { return min_.has_value() || max_.has_value(); }

	protected:
		std::optional<ValueType> min_;
		std::optional<ValueType> max_;
	};

	/**
	 * A interval with fuzzy boundaries described by a range [min,max].
	 * @tparam ValueType the value type of the interval.
	 */
	template <class ValueType> class FuzzyInterval {
	public:
		/**
		 * @param minRange the range where the interval starts
		 * @param maxRange the range where the interval ends
		 */
		FuzzyInterval(const Range<ValueType> &minRange, const Range<ValueType> &maxRange)
				: minRange_(minRange), maxRange_(maxRange) {}

		/**
		 * @return the range where the interval starts
		 */
		const Range<ValueType>& minRange() const { return minRange_; }

		/**
		 * @return the range where the interval ends
		 */
		const Range<ValueType>& maxRange() const { return maxRange_; }

		/**
		 * @return true if this interval does not contain any elements.
		 */
		bool empty() const {
			return minRange_.empty() || maxRange_.empty() ||
				   (maxRange_.max().has_value() && minRange_.min().has_value() &&
				    maxRange_.max().value() < minRange_.min().value());
		}

		/**
		 * @param elem a confidenceInterval value.
		 * @return true if the value falls within this interval.
		 */
		bool contains(const ValueType &elem) const {
			return (minRange_.min().has_value() && elem < minRange_.min().value()) ||
				   (maxRange_.max().has_value() && maxRange_.max().value() < elem);
		}

		/**
		 * A fuzzy interval is thought to be more general than another
		 * if its min and max range contains the respective range of the other.
		 * @param other another interval
		 * @return true if other is a specialization of this
		 */
		bool isMoreGeneralThan(const FuzzyInterval<ValueType> &other) const {
			return minRange_.contains(other.minRange_) &&
				   maxRange_.contains(other.maxRange_);
		}

	protected:
		Range<ValueType> minRange_;
		Range<ValueType> maxRange_;
	};

	/**
	 * A point in time.
	 */
	class TimePoint {
	public:
		/**
		 * @param value time in seconds
		 */
		TimePoint(const double& value);

		/**
		 * @return the current system time
		 */
		static TimePoint now();

		/**
		 * @return the value of this time point in seconds
		 */
		const double& value() const { return value_; }

		/**
		 * @param other another time point
		 * @return true if this time point occurs earlier than another one.
		 */
		bool operator<(const TimePoint& other) const;

		/**
		 * @param other another time point
		 * @return true if this time point equals the other in nanosecond resolution.
		 */
		bool operator==(const TimePoint& other) const;

	protected:
		double value_;
	};

	/**
	 * A fuzzy time interval where start and end time point lie within a range.
	 */
	class TimeInterval : public FuzzyInterval<TimePoint> {
	public:
		/**
		 * @param sinceRange the time range where the interval starts
		 * @param untilRange the time range where the interval ends
		 */
		TimeInterval(const Range<TimePoint> &sinceRange, const Range<TimePoint> &untilRange);

		/**
		 * @return a time interval without further constraints on begin and end time point of the interval.
		 */
		static const TimeInterval& anytime();

		/**
		 * @return a time interval that at least intersects with the current time.
		 */
		static TimeInterval currently();

		/**
		 * Intersect this time interval with another one.
		 * @param other another time interval.
		 */
		std::shared_ptr<TimeInterval> intersectWith(const TimeInterval &other) const;
	};

	/**
	 * The amount of confidenceInterval for something represented within the range [0,1].
	 */
	class ConfidenceValue {
	public:
		/**
		 * @param value the confidenceInterval value in the range [0,1]
		 */
		explicit ConfidenceValue(double value);

		/**
		 * @return the maximum confidenceInterval value.
		 */
		static const ConfidenceValue& max();

		/**
		 * @return the minimum confidenceInterval value.
		 */
		static const ConfidenceValue& min();

		/**
		 * @return the confidenceInterval value in the range [0,1]
		 */
		const double& value() const { return value_; }

		/**
		 * @param other another confidenceInterval value.
		 * @return true if this confidenceInterval value is smaller than the other.
		 */
		bool operator<(const ConfidenceValue& other) const;

		/**
		 * @param other another confidenceInterval value.
		 * @return true if both values are the same
		 */
		bool operator==(const ConfidenceValue& other) const;

	protected:
		double value_;
	};

	/**
	 * A fuzzy confidenceInterval interval where min and end max confidence lie within a range.
	 */
	class ConfidenceInterval : public FuzzyInterval<ConfidenceValue> {
	public:
		/**
		 * @param minRange the value range for the minimum confidenceInterval
		 * @param maxRange the value range for the maximum confidenceInterval
		 */
		ConfidenceInterval(const Range<ConfidenceValue> &minRange, const Range<ConfidenceValue> &maxRange);

		/**
		 * @return an interval that only includes the maximum confidenceInterval value.
		 */
		static const ConfidenceInterval& certain();

		/**
		 * @return a confidenceInterval interval without any further constraints on min and max confidence values.
		 */
		static const ConfidenceInterval& any();

		/**
		 * @param value the minimum confidenceInterval value
		 * @return a confidence interval including all confidenceInterval values larger than the one provided
		 */
		static ConfidenceInterval at_least(const ConfidenceValue &value);
	};
	
	/**
	 * A query represented by a propositional formula.
	 */
	class Query {
	public:
		/**
		 * @formula the formula associated to this query.
		 */
		explicit Query(const FormulaPtr &formula);
		
		/**
		 * @predicate the predicate that is queried.
		 */
		explicit Query(const std::shared_ptr<Predicate> &predicate);

		/**
		 * @return the formula associated to this query.
		 */
		const FormulaPtr& formula() const { return formula_; }
		
		/**
		 * Replaces variables in the query with terms based on a mapping provided in the argument.
		 * @sub a mapping from variables to terms.
		 * @return the new query created.
		 */
		std::shared_ptr<Query> applySubstitution(const Substitution &sub) const;

		/**
		 * Assigns a time interval to this query indicating that solutions
		 * should only be generated that are valid within this interval.
		 * @param timeInterval the time interval of this query.
		 */
		void setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval);

		/**
		 * Assigns a confidenceInterval interval to this query indicating that solutions
		 * should only be generated that are valid within this interval.
		 * @param confidence the confidenceInterval interval of this query.
		 */
		void setConfidenceInterval(const std::shared_ptr<ConfidenceInterval> &confidenceInterval);

		/**
		 * @return an optional time interval of this query.
		 */
		const std::optional<const TimeInterval*>& timeInterval() const { return o_timeInterval_; }

		/**
		 * @return an optional confidenceInterval interval of this query.
		 */
		const std::optional<const ConfidenceInterval*>& confidenceInterval() const { return o_confidenceInterval_; }

	protected:
		const std::shared_ptr<Formula> formula_;
		std::shared_ptr<TimeInterval> timeInterval_;
		std::shared_ptr<ConfidenceInterval> confidenceInterval_;
		std::optional<const TimeInterval*> o_timeInterval_;
		std::optional<const ConfidenceInterval*> o_confidenceInterval_;
		friend class QueryInstance;
	};

	/**
	 * The instantiation of a predicate in a reasoner module.
	 */
	class PredicateInstance {
	public:
		/**
		 * @param reasonerModule reasoner module term
		 * @param predicate a predicate instance
		 */
		PredicateInstance(const std::shared_ptr<StringTerm> &reasonerModule,
						  const std::shared_ptr<Predicate> &predicate)
		: reasonerModule_(reasonerModule), predicate_(predicate) {}

		/**
		 * @return reasoner module term
		 */
		const std::shared_ptr<StringTerm>& reasonerModule() const { return reasonerModule_; }

		/**
		 * @return a predicate instance
		 */
		const std::shared_ptr<Predicate>& predicate() const { return predicate_; }

	protected:
		const std::shared_ptr<StringTerm> reasonerModule_;
		const std::shared_ptr<Predicate> predicate_;
	};

	/**
	 * The result of query evaluation.
	 * A result indicates that the evaluation succeeded, i.e.,
	 * that a reasoner was able to find an instance of the query that is true.
	 */
	class QueryResult {
	public:
		QueryResult();

		/**
		 * Copy another result.
		 * Modification of the constructed result won't affect the copied one.
		 * @param other another query result.
		 */
		QueryResult(const QueryResult &other);

		/**
		 * @return a positive result without additional constraints.
		 */
		static const std::shared_ptr<const QueryResult>& emptyResult();

		/**
		 * Adds to this result a substitution of a variable with a term.
		 * @param var a variable
		 * @param term a term
		 */
		void substitute(const Variable &var, const TermPtr &term);

		/**
		 * @param var a variable.
		 * @return true is this solution substitutes the variable
		 */
		bool hasSubstitution(const Variable &var);

		/**
		 * @return a mapping from variables to terms.
		 */
		const SubstitutionPtr& substitution() const { return substitution_; }

		/**
		 * @param reasonerModule the reasoner module the inferred the instance
		 * @param instance an instance of a query predicate
		 */
		void addPredicate(const std::shared_ptr<StringTerm> &reasonerModule,
						  const std::shared_ptr<Predicate> &predicate);

		/**
		 * A list of all query predicates that were instantiated to reach
		 * this solution. Only predicates that appear in the user query are
		 * included in this list.
		 * @return instantiated predicates.
		 */
		const std::list<PredicateInstance>& predicates() const { return predicates_; }

		/**
		 * Assigns a time interval to this solution indicating that the solution
		 * is only valid in a restricted time frame.
		 * @param timeInterval the time interval of the result being valid.
		 */
		void setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval);

		/**
		 * Assigns a confidenceInterval value to this solution.
		 * @param confidence a confidenceInterval value of the result being valid.
		 */
		void setConfidenceValue(const std::shared_ptr<ConfidenceValue> &confidence);

		/**
		 * @return an optional time interval of the result being valid.
		 */
		const std::optional<const TimeInterval*>& timeInterval() const { return o_timeInterval_; }

		/**
		 * @return an optional confidenceInterval value of the result being valid.
		 */
		const std::optional<const ConfidenceValue*>& confidence() const { return o_confidence_; }

		/**
		 * Merge another query result into this one.
		 * A merge failure is indicated by the return value, e.g. in case
		 * both substitutions cannot be unified false is returned.
		 * @param other another query result.
		 * @param changes used to make the merge operation reversible, can be null.
		 * @return false if merge is not possible.
		 */
		bool combine(const std::shared_ptr<const QueryResult> &other, Reversible *changes=nullptr);

	protected:
		SubstitutionPtr substitution_;
		std::list<PredicateInstance> predicates_;
		std::shared_ptr<TimeInterval> timeInterval_;
		std::shared_ptr<ConfidenceValue> confidence_;
		std::optional<const TimeInterval*> o_timeInterval_;
		std::optional<const ConfidenceValue*> o_confidence_;

		bool combineConfidence(const std::shared_ptr<ConfidenceValue> &otherConfidence);

		bool combineTimeInterval(const std::shared_ptr<TimeInterval> &otherTimeInterval,
								 Reversible *changes= nullptr);

		friend class QueryInstance;
	};
	// alias
	using QueryResultPtr = std::shared_ptr<const QueryResult>;
	
	/**
	 * A stream of query results.
	 * The only way to write to a stream is by creating a channel.
	 */
	class QueryResultStream {
	public:
		QueryResultStream();
		~QueryResultStream();
		QueryResultStream(const QueryResultStream&) = delete;
		
		/**
		 * Find out if a message indicates the end-of-stream (EOS).
		 * @msg a QueryResult pointer.
		 * @return true if the result indicates EOS.
		 */
		static bool isEOS(const QueryResultPtr &msg);
		
		/**
		 * Get the end-of-stream (eos) message.
		 * @return the eos message.
		 */
		static QueryResultPtr& eos();
		
		/**
		 * Get the begin-of-stream (bos) message.
		 * This is basically an empty substitution mapping.
		 * @return the bos message.
		 */
		static QueryResultPtr& bos();
		
		/**
		 * Close the stream.
		 * This will push an EOS message, and all future
		 * attempts to push a non EOS message will cause an error.
		 * Once closed, a stream cannot be opened again.
		 * Note that a stream auto-closes once it has received EOS
		 * messages from all of its input channels.
		 */
		void close();
		
		/**
		 * @return true if opened.
		 */
		bool isOpened() const;
		
		/**
		 * An input channel of a stream.
		 */
		class Channel {
		public:
			/**
			 * @param stream the query result stream associated to this channel.
			 */
			explicit Channel(const std::shared_ptr<QueryResultStream> &stream);

			~Channel();

			/**
			 * Cannot be copy-assigned.
			 */
			Channel(const Channel&) = delete;

			/**
			 * Create a new stream channel.
			 * Note that this will generate an error in case the stream
			 * is closed already.
			 * @return a new stream channel
			 */
			static std::shared_ptr<Channel> create(const std::shared_ptr<QueryResultStream> &stream);
			
			/**
			 * Push a QueryResult into this channel.
			 * @msg a QueryResult pointer.
			 */
			void push(const QueryResultPtr &msg);
			
			/**
			 * Close the channel.
			 */
			void close();
		
			/**
			 * @return true if opened.
			 */
			bool isOpened() const;
			
			/**
			 * @return the id of this channel
			 */
			uint32_t id() const;
			
		protected:
			// the stream of this channel
			const std::shared_ptr<QueryResultStream> stream_;
			// iterator of this channel withing the stream
			std::list<std::shared_ptr<Channel>>::iterator iterator_;
			// flag indicating whether channel is open (i.e., no EOS received so far)
			std::atomic<bool> isOpened_;
			
			friend class QueryResultStream;
		};

	protected:
		std::list<std::shared_ptr<Channel>> channels_;
		std::atomic<bool> isOpened_;
		std::mutex channel_mutex_;

		virtual void push(const Channel &channel, const QueryResultPtr &msg);
		
		virtual void push(const QueryResultPtr &msg) = 0;
	};
	
	/**
	 * A queue of QueryResult objects.
	 */
	class QueryResultQueue : public QueryResultStream {
	public:
		QueryResultQueue();
		~QueryResultQueue();
		
		/**
		 * Get the front element of this queue without removing it.
		 * This will block until the queue is non empty.
		 * @return the front element of the queue.
		 */
		QueryResultPtr& front();
		
		/**
		 * Remove the front element of this queue.
		 */
		void pop();
		
		/**
		 * Get front element and remove it from the queue.
		 * @return the front element of the queue.
		 */
		QueryResultPtr pop_front();

	protected:
		std::queue<QueryResultPtr> queue_;
		std::condition_variable queue_CV_;
		std::mutex queue_mutex_;
		
		// Override QueryResultStream
		void push(const QueryResultPtr &item) override;
		void pushToQueue(const QueryResultPtr &item);
	};
	
	/**
	 * A broadcaster of query results.
	 */
	class QueryResultBroadcaster : public QueryResultStream {
	public:
		QueryResultBroadcaster();
		~QueryResultBroadcaster();
		
		/**
		 * Add a subscriber to this broadcast.
		 * The subscriber will receive input from the broadcast after this call.
		 * @subscriber a query result stream.
		 */
		void addSubscriber(const std::shared_ptr<Channel> &subscriber);
		
		/**
		 * Remove a previously added subscriber.
		 * @subscriber a query result stream.
		 */
		void removeSubscriber(const std::shared_ptr<Channel> &subscriber);

	protected:
		std::list<std::shared_ptr<Channel>> subscribers_;
		
		// Override QueryResultStream
		void push(const QueryResultPtr &msg) override;
		void pushToBroadcast(const QueryResultPtr &msg);
	};
	
	// alias
	using QueryResultBuffer = std::map<uint32_t, std::list<QueryResultPtr>>;
	
	/**
	 * Combines multiple query result streams, and broadcasts each
	 * combination computed.
	 * This is intended to be used for parallel evaluation of
	 * sub-goals within a query.
	 */
	class QueryResultCombiner : public QueryResultBroadcaster {
	public:
		QueryResultCombiner();
	
	protected:
		QueryResultBuffer buffer_;
		std::mutex buffer_mutex_;
		
		// Override QueryResultStream
		void push(const Channel &channel, const QueryResultPtr &msg) override;
		
		void genCombinations(uint32_t pushedChannelID,
			QueryResultBuffer::iterator it,
			std::shared_ptr<QueryResult> &combinedResult);
	};

	/**
	 * An instantiation of a query, i.e. where some variables may be substituted by terms.
	 */
	class QueryInstance {
	public:
		/**
		 * @param uninstantiatedQuery a query that may contain variables
		 * @param outputChannel an output channel where solutions are pushed
		 * @param partialResult a partial result from sub-queries evaluated before
		 */
		QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
					  const std::shared_ptr<QueryResultStream::Channel> &outputChannel,
					  const std::shared_ptr<const QueryResult> &partialResult);
		/**
		 * @param uninstantiatedQuery a query that may contain variables
		 * @param outputChannel an output channel where solutions are pushed
		 */
		QueryInstance(const std::shared_ptr<const Query> &uninstantiatedQuery,
					  const std::shared_ptr<QueryResultStream::Channel> &outputChannel);

		/**
		 * @return an instane of the input query.
		 */
		std::shared_ptr<const Query> create();

		/**
		 * Push a new solution for the instantiated query into the QA pipeline.
		 * @param solution a query solution
		 */
		void pushSolution(const std::shared_ptr<QueryResult> &solution);

		/**
		 * Push EOS message indicating that no more solutions will be generated.
		 */
		void pushEOS();

		/**
		 * @return the uninstantiated query.
		 */
		const std::shared_ptr<const Query>& uninstantiatedQuery() const { return uninstantiatedQuery_; }

		/**
		 * @return an optional time interval of this query.
		 */
		const std::optional<const TimeInterval*>& timeInterval() const;

		/**
		 * @return an optional confidenceInterval interval of this query.
		 */
		const std::optional<const ConfidenceInterval*>& confidenceInterval() const;

	protected:
		std::shared_ptr<const Query> uninstantiatedQuery_;
		std::shared_ptr<const QueryResult> partialResult_;
		std::shared_ptr<QueryResultStream::Channel> outputChannel_;
	};
	using QueryInstancePtr = std::shared_ptr<QueryInstance>;
	
	/**
	 * A querying-related runtime error.
	 */
	class QueryError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args> QueryError(const char *fmt, Args&& ... args)
		: std::runtime_error(fmt::format(fmt, args...))
		{}

		/**
		 * @param erroneousQuery the query that caused an error
		 * @param errorTerm a term denoting the error
		 */
		QueryError(const Query &erroneousQuery, const Term &errorTerm);
	
	protected:
		
		static std::string formatErrorString(const Query &erroneousQuery, const Term &errorTerm);
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::ConfidenceValue& confidence);
	std::ostream& operator<<(std::ostream& os, const knowrob::TimePoint& tp);

	std::ostream& operator<<(std::ostream& os, const knowrob::Formula& phi);
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q);
	std::ostream& operator<<(std::ostream& os, const knowrob::QueryResult& solution);

	std::ostream& operator<<(std::ostream& os, const knowrob::Range<knowrob::TimePoint>& t);
	std::ostream& operator<<(std::ostream& os, const knowrob::TimeInterval& ti);
}

#endif //KNOWROB_QUERIES_H_
