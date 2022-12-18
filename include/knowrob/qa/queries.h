/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_QUERIES_H__
#define __KNOWROB_QUERIES_H__

// STD
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <string>
#include <atomic>
// KnowRob
#include <knowrob/ThreadPool.h>
#include <knowrob/lang/terms.h>

namespace knowrob {
	/** The type of a formula.
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
	
	/** A propositional formula.
	 * Note that all formulas are immutable.
	 */
	class Formula {
	public:
		/**
		 * @type the type of the formula.
		 */
		Formula(const FormulaType &type);
		
		/**
		 * @return the type of this formula.
		 */
		FormulaType type() const { return type_; }
		
		/** Is this formula free of subformulas?
		 *
		 * @return true if this formula is atomic.
		 */
		bool isAtomic() const;
		
		/**
		 * @return true if this formula contains no free variables.
		 */
		virtual bool isGround() const = 0;
		
		/** Replaces variables in the formula with terms.
		 * @sub a substitution mapping.
		 * @return the created formula.
		 */
		virtual std::shared_ptr<Formula> applySubstitution(const Substitution &sub) const = 0;
		
		/** Write the formula into an ostream.
		 */
		virtual void write(std::ostream& os) const = 0;
	
	protected:
		const FormulaType type_;
	};
	
	// alias declaration
	using FormulaPtr = std::shared_ptr<Formula>;
	
	/** A predicate formula.
	 */
	class PredicateFormula : public Formula {
	public:
		/**
		 * @predicate a predicate reference.
		 */
		PredicateFormula(const std::shared_ptr<Predicate> &predicate);
		
		/**
		 * @return the predicate associated to this formula.
		 */
		const std::shared_ptr<Predicate>& predicate() const { return predicate_; }
		
		// Override Formula
		bool isGround() const;
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const;
		
		// Override Formula
		void write(std::ostream& os) const;
		
	protected:
		const std::shared_ptr<Predicate> predicate_;
	};
	
	/** A formula with sub-formulas linked via logical connectives.
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
		bool isGround() const;
		
		// Override Formula
		void write(std::ostream& os) const;
	
	protected:
		const std::vector<FormulaPtr> formulae_;
		const bool isGround_;
		
		ConnectiveFormula(const ConnectiveFormula &other, const Substitution &sub);
		
		bool isGround1() const;
		
		std::vector<FormulaPtr> applySubstitution1(
			const std::vector<FormulaPtr> &otherFormulas,
			const Substitution &sub) const;
	};
	
	/** A conjunctive expression.
	 */
	class ConjunctionFormula : public ConnectiveFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		ConjunctionFormula(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const { return "\u2227"; }
		
	protected:
		ConjunctionFormula(const ConjunctionFormula &other, const Substitution &sub);
	};
	
	/** A disjunctive expression.
	 */
	class DisjunctionFormula : public ConnectiveFormula {
	public:
		/**
		 * @formulae list of sub-formulas.
		 */
		DisjunctionFormula(const std::vector<FormulaPtr> &formulae);
		
		// Override Formula
		FormulaPtr applySubstitution(const Substitution &sub) const;
		
		// Override ConnectiveFormula
		const char* operator_symbol() const { return "\u2228"; }
	
	protected:
		DisjunctionFormula(const DisjunctionFormula &other, const Substitution &sub);
	};
	
	/**
	 * A query represented by a propositional formula.
	 */
	class Query {
	public:
		/**
		 * @formula the formula associated to this query.
		 */
		Query(const FormulaPtr &formula);
		
		/**
		 * @predicate the predicate that is queried.
		 */
		Query(const std::shared_ptr<Predicate> &predicate);

		/**
		 * @return the formula associated to this query.
		 */
		const FormulaPtr& formula() const { return formula_; }
		
		/** Replaces variables in the query with terms based on a mapping provided in the argument.
		 * @sub a mapping from variables to terms.
		 * @return the new query created.
		 */
		std::shared_ptr<Query> applySubstitution(const Substitution &sub) const;

	protected:
		const std::shared_ptr<Formula> formula_;
		
		FormulaPtr copyFormula(const FormulaPtr &phi);
		TermPtr copyTerm(const TermPtr &t);
	};
	
	// aliases
	using QueryResult = Substitution;
	using QueryResultPtr = SubstitutionPtr;
	// forward declaration
	class QueryResultBroadcast;
	
	/**
	 * A stream of query results.
	 * The only way to write to a stream is by creating a channel.
	 */
	class QueryResultStream {
	public:
		QueryResultStream();
		~QueryResultStream();
		
		/** Find out if a message indicates the end-of-stream (EOS).
		 * @msg a QueryResult pointer.
		 * @return true if the result indicates EOS.
		 */
		static bool isEOS(const QueryResultPtr &msg);
		
		/** Get the end-of-stream (eos) message. 
		 * @return the eos message.
		 */
		static QueryResultPtr& eos();
		
		/** Get the begin-of-stream (bos) message.
		 * This is basically an empty substitution mapping.
		 * @return the bos message.
		 */
		static QueryResultPtr& bos();
		
		/** Close the stream.
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
		
		/** An input channel of a stream.
		 */
		class Channel {
		public:
			Channel(QueryResultStream *stream);
			
			~Channel();
			
			/** Push a QueryResult into this channel.
			 * @msg a QueryResult pointer.
			 */
			void push(const QueryResultPtr &msg);
			
			/** Close the channel.
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
			QueryResultStream *stream_;
			std::list<std::shared_ptr<Channel>>::iterator iterator_;
			std::atomic<bool> isOpened_;
			
			friend class QueryResultStream;
			
			Channel(const Channel&) = delete;
		};
		
		/** Create a new stream channel.
		 * Note that this will generate an error in case the stream
		 * is closed already.
		 * @return a new stream channel
		 */
		std::shared_ptr<QueryResultStream::Channel> createChannel();

	protected:
		std::list<std::shared_ptr<Channel>> channels_;
		std::atomic<bool> isOpened_;
		std::mutex channel_mutex_;
		
		virtual void push(const Channel &channel, const QueryResultPtr &msg);
		
		virtual void push(const QueryResultPtr &msg) = 0;
		
		// copying streams is not allowed
		QueryResultStream(const QueryResultStream&) = delete;
	};
	
	/** A queue of QueryResult objects.
	 */
	class QueryResultQueue : public QueryResultStream {
	public:
		QueryResultQueue();
		~QueryResultQueue();
		
		/** Get the front element of this queue without removing it.
		 * This will block until the queue is non empty.
		 * @return the front element of the queue.
		 */
		QueryResultPtr& front();
		
		/** Remove the front element of this queue.
		 */
		void pop();
		
		/** Get front element and remove it from the queue.
		 * @return the front element of the queue.
		 */
		QueryResultPtr pop_front();

	protected:
		std::queue<QueryResultPtr> queue_;
		std::condition_variable queue_CV_;
		std::mutex queue_mutex_;
		
		// Override QueryResultStream
		void push(const QueryResultPtr &item);
	};
	
	/** A broadcaster of query results.
	 */
	class QueryResultBroadcaster : public QueryResultStream {
	public:
		QueryResultBroadcaster();
		~QueryResultBroadcaster();
		
		/** Add a subscriber to this broadcast.
		 * The subscriber will receive input from the broadcast after this call.
		 * @subscriber a query result stream.
		 */
		void addSubscriber(const std::shared_ptr<Channel> &subscriber);
		
		/** Remove a previously added subscriber.
		 * @subscriber a query result stream.
		 */
		void removeSubscriber(const std::shared_ptr<Channel> &subscriber);

	protected:
		std::list<std::shared_ptr<Channel>> subscribers_;
		
		// Override QueryResultStream
		void push(const QueryResultPtr &msg);
	};
	
	// alias
	using QueryResultBuffer = std::map<uint32_t, std::list<QueryResultPtr>>;
	
	/** Combines multiple query result streams.
	 * This is intended to be used for parallel evaluation of
	 * subgoals within a query.
	 */
	class QueryResultCombiner : public QueryResultBroadcaster {
	public:
		QueryResultCombiner();
	
	protected:
		QueryResultBuffer buffer_;
		std::mutex buffer_mutex_;
		
		// Override QueryResultStream
		void push(const Channel &channel, const QueryResultPtr &msg);
		
		void genCombinations(uint32_t pushedChannelID,
			QueryResultBuffer::iterator it,
			SubstitutionPtr &combination);
	};
	
	/** An error during query processing.
	 */
	class QueryError : public std::runtime_error {
	public:
		QueryError(const Query &erroneousQuery, const Term &errorTerm);
		
		QueryError(const std::string& what = "");
	
	protected:
		
		std::string formatErrorString(const Query &erroneousQuery, const Term &errorTerm);
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Formula& phi);
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q);
}

#endif //__KNOWROB_QUERIES_H__
