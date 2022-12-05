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
#include <stdexcept>
// KnowRob
#include <knowrob/knowrob.h>
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
	
	/** An expression in the querying language.
	 */
	class Formula {
	public:
		/** Default constructor.
		 * @type the type of the formula.
		 */
		Formula(const FormulaType &type);
		
		/** Get the formula type.
		 *
		 * @return the type of this formula.
		 */
		FormulaType type() const { return type_; }
		
		/** Is this formula atomic?
		 *
		 * @return true if this formula is atomic.
		 */
		bool isAtomic() const;
	
	protected:
		FormulaType type_;
	};
	
	/** A predicate expression in the querying language.
	 */
	class PredicateFormula : public Formula {
	public:
		/** Default constructor.
		 * @predicate a predicate.
		 */
		PredicateFormula(const std::shared_ptr<Predicate> &predicate)
		: Formula(FormulaType::PREDICATE),
		  predicate_(predicate) {}
		
		/** Get the predicate associated to this formula.
		 *
		 * @return the predicate.
		 */
		const std::shared_ptr<Predicate>& predicate() const { return predicate_; }
		
	protected:
		std::shared_ptr<Predicate> predicate_;
	};
	
	/** An expression using logical connectives.
	 */
	class ConnectiveFormula : public Formula {
	public:
		/** Default constructor.
		 * @type the type of the formula.
		 * @formulae list of connected formulae.
		 */
		ConnectiveFormula(const FormulaType &type,
			const std::vector<std::shared_ptr<Formula>> &formulae)
		: Formula(type),
		  formulae_(formulae){}
		
		/** Get the sub-formulae associated to this formula.
		 *
		 * @return the sub-formulae.
		 */
		const std::vector<std::shared_ptr<Formula>>& formulae() const { return formulae_; }
	
	protected:
		std::vector<std::shared_ptr<Formula>> formulae_;
	};
	
	/** A conjunctive expression.
	 */
	class ConjunctionFormula : public ConnectiveFormula {
	public:
		/** Default constructor.
		 * @formulae list of connected formulae.
		 */
		ConjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
		: ConnectiveFormula(FormulaType::CONJUNCTION, formulae){}
	};
	
	/** A disjunctive expression.
	 */
	class DisjunctionFormula : public ConnectiveFormula {
	public:
		/** Default constructor.
		 * @formulae list of connected formulae.
		 */
		DisjunctionFormula(const std::vector<std::shared_ptr<Formula>> &formulae)
		: ConnectiveFormula(FormulaType::DISJUNCTION, formulae){}
	};
	
	/**
	 * A query that is represented by a Formula.
	 */
	class Query {
	public:
		/** Default constructor.
		 */
		Query(const std::shared_ptr<Formula> &formula)
		: formula_(formula){}
		
		/** Create a simple query about a single predicate.
		 */
		Query(const std::shared_ptr<Predicate> &predicate)
		: formula_(new PredicateFormula(predicate)) {}

		/** Get the formula associated to this query.
		 * @return the formula.
		 */
		const std::shared_ptr<Formula>& formula() const { return formula_; }

	protected:
		std::shared_ptr<Formula> formula_;
	};

	/**
	 */
	class QueryResult {
	public:
		static const QueryResult *NO_SOLUTION;
		
		QueryResult();
		
		/**
		 */
		bool hasSolution();
		
		/** Add a substitution of a variable with a term.
		 *
		 * @var a variable.
		 * @term a term.
		 */
		void set(const Variable &var, const std::shared_ptr<Term> &term);
		
		/** Get the substitution of a variable.
		 *
		 * Note: default is to map a variable to itself.
		 *
		 * @var a variable.
		 * @term a term.
		 */
		std::shared_ptr<Term> get(const Variable &var) const;

	private:
		std::map<Variable, std::shared_ptr<Term>> mapping_;
		bool hasSolution_;
	};
	
	using QueryResultPtr = std::shared_ptr<QueryResult>;
	
	// forward declaration
	class QueryResultBroadcast;
	
	/**
	 * A stream of QueryResult objects that provides an interface
	 * to push additional objects into the stream.
	 */
	class QueryResultStream {
	public:
		QueryResultStream();
		~QueryResultStream();
		
		// copy constructor is not supported for QueryResultStream
		QueryResultStream(const QueryResultStream&) = delete;
		
		/** Find out if a message indicates the end-of-stream (EOS).
		 * @msg a QueryResult pointer.
		 * @return true if the result indicates EOS.
		 */
		static bool isEOS(QueryResultPtr &msg);
		
		/** Push an additional QueryResult into this stream.
		 * @msg a QueryResult pointer.
		 */
		virtual void push(QueryResultPtr &msg) = 0;

	protected:
		std::list<QueryResultBroadcast*> subscriptions_;
	};
	
	/** A broadcaster of QueryResult objects to a list of subscribers.
	 */
	class QueryResultBroadcast : public QueryResultStream {
		QueryResultBroadcast();
		~QueryResultBroadcast();
		
		/** Add a subscriber to this broadcast.
		 * The subscriber will receive input from the broadcast after this call.
		 * @subscriber a query result stream.
		 */
		void addSubscriber(QueryResultStream *subscriber);
		
		/** Remove a previously added subscriber.
		 * @subscriber a query result stream.
		 */
		void removeSubscriber(QueryResultStream *subscriber);
		
		// Override QueryResultStream::push
		void push(QueryResultPtr &item);

	protected:
		std::list<QueryResultStream*> subscribers_;
	};
	
	/** A queue of QueryResult objects.
	 */
	class QueryResultQueue : public QueryResultStream {
	public:
		QueryResultQueue();
		
		/** Get the front element of this queue without removing it.
		 * This will block until the queue is non empty.
		 * @return the front element of the queue.
		 */
		QueryResultPtr& front();
		
		/** Remove the front element of this queue.
		 */
		void pop();
		
		// Override QueryResultStream::push
		void push(QueryResultPtr &item);

	protected:
		std::mutex mutex_;
		std::condition_variable cond_var_;
		std::queue<QueryResultPtr> queue_;
	};
}

#endif //__KNOWROB_QUERIES_H__
