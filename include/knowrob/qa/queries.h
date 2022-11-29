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
// boost
#include <boost/shared_ptr.hpp>
// KnowRob
#include <knowrob/lang/terms.h>

namespace knowrob {
	typedef QueryResultQueue Queue<boost::shared_ptr<QueryResult>>;
	
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
		
		/** Is this formular atomic?
		 *
		 * @return true if this formula is atomic.
		 */
		bool isAtomic() const;
		
		/** Get the set of variables in this formula.
		 *
		 * Free as well as bounded variables are included in the set.
		 *
		 * @return the set of variables that appear in this formula.
		 */
		std::set<Variable> getVariables();
	
	protected:
		virtual void readVariables(std::set<Variable> &output) = 0;
	};
	
	/** A predicate expression in the querying language.
	 */
	class PredicateFormula : public Formula {
	public:
		/** Default constructor.
		 * @predicate a predicate.
		 */
		PredicateFormula(const Predicate &predicate)
		: Formula(FormulaType::PREDICATE), p_(p) {}
		
		/** Get the predicate associated to this formula.
		 *
		 * @return the predicate.
		 */
		const Predicate& predicate() const { return predicate_; }
		
	protected:
		Predicate predicate_;
		
		void readVariables(std::set<Variable> &output);
	};
	
	/** An expression using logical connectives.
	 */
	class ConnectiveFormula : public Formula {
	public:
		/** Default constructor.
		 * @type the type of the formula.
		 * @formulae list of connected formulae.
		 */
		ConnectiveFormula(const FormulaType &type, const std::vector<Formula> &formulae)
		: Formula(type), formulae_(formulae){}
		
		/** Get the sub-formulae associated to this formula.
		 *
		 * @return the sub-formulae.
		 */
		const std::vector<Formula>& formulae() const { return formulae_; }
	
	protected:
		std::vector<Formula> formulae_;
	
		void readVariables(std::set<Variable> &output);
	};
	
	/** A conjunctive expression.
	 */
	class ConjunctionFormula : public ConnectiveFormula {
	public:
		/** Default constructor.
		 * @formulae list of connected formulae.
		 */
		ConjunctionFormula(const std::vector<Formula> &formulae)
		: ConnectiveFormula(FormulaType::CONJUNCTION, formulae){}
	};
	
	/** A disjunctive expression.
	 */
	class DisjunctionFormula : public ConnectiveFormula {
	public:
		/** Default constructor.
		 * @formulae list of connected formulae.
		 */
		DisjunctionFormula(const std::vector<Formula> &formulae)
		: ConnectiveFormula(FormulaType::DISJUNCTION, formulae){}
	};
	
	/**
	 * A query that is represented by a Formula.
	 */
	class Query {
	public:
		/** Default constructor.
		 */
		Query(const Formula &formula)
		: formula_(formula){}
		
		/** Create a simple query about a single predicate.
		 */
		Query(const Predicate &predicate)
		: formula_(PredicateFormula(predicate)) {}

		/**
		 */
		const Formula& formula() const { return formula_; }

	protected:
		Formula formula_;
	};
	
	/** An interface for constructing query objects from strings.
	 * Also provides an interface to generate string representations
	 * of queries.
	 */
	class IQueryParser {
	public:
		/** Get the identifier of the language supported by the parser.
		 *
		 * @return the language identifier, e.g. "prolog" for the PrologQueryParser.
		 */
		virtual const std::string& getLanguageIdentifier() const = 0;
		
		/** Parse a Query object from a query string encoded in the language supported by the parser.
		 *
		 * @queryString an expression in the language supported by the parser.
		 * @return the newly constructed
		 */
		virtual boost::shared_ptr<Query> fromString(const std::string &queryString) = 0;
		
		/** Convert a Query object into an expression of the language supported by the parser.
		 *
		 * @query a query object
		 * @return the query encoded as an expression in the language of the parser.
		 */
		virtual std::string toString(const Query &query) = 0;
	};

	/** A query parser supporting the Prolog language for encoding the query.
	 */
	class PrologQueryParser : public IQueryParser {
	public:
		// Override
		const std::string& getLanguageIdentifier() const;
		
		// Override
		std::string toString(const Query &query);
		
		// Override
		boost::shared_ptr<Query> fromString(const std::string &queryString);
	};
	
	/**
	 */
	class Substitution {
	public:
		Substitution();
		~Substitution();
		
		void set(Variable var, Term assignment);
		
		const Term& get(const Variable &var) const;
		
		/** Apply the substitution to a term.
		 */
		Term apply(const Term &term);
		
	private:
		std::map<Variable,Term> assignments_;
	};

	/**
	 */
	class QueryResult {
	public:
		static const QueryResult& noSolution();
		
		QueryResult(const boost::shared_ptr<Substitution> &substitution);
		~QueryResult();
		
		bool hasSolution()
		{ return hasSolution_; }
		
		const boost::shared_ptr<Substitution>& substitution()
		{ return substitution_; }

	private:
		boost::shared_ptr<Substitution> substitution_;
		bool hasSolution_;
		
		// private constructor for "noSolution" QueryResult
		QueryResult();
	};
}

#endif //__KNOWROB_QUERIES_H__
