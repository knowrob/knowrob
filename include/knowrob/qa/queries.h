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
		
		/** Get the set of variables in this formula.
		 *
		 * Free as well as bounded variables are included in the set.
		 *
		 * @return the set of variables that appear in this formula.
		 */
		std::set<Variable> getVariables();
	
		virtual void readVariables(std::set<Variable> &output) = 0;
	
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
		
		void readVariables(std::set<Variable> &output);
		
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
	
		void readVariables(std::set<Variable> &output);
	
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
	class ParserError : public std::runtime_error {
	public:
		/**
		 */
		ParserError(const std::string& what = "") : std::runtime_error(what) {}
	};
	
	/** An interface for constructing query objects from strings.
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
		 * @return the newly constructed query object.
		 */
		virtual Query fromString(const std::string &queryString) = 0;
	};
	
	/** An interface for constructing strings from query objects.
	 */
	class IQueryFormatter {
	public:
		/** Get the identifier of the language supported by the parser.
		 *
		 * @return the language identifier, e.g. "prolog" for the PrologQueryParser.
		 */
		virtual const std::string& getLanguageIdentifier() const = 0;
		
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
		Query fromString(const std::string &queryString);
	};
	
	/** A mapping from variables to terms.
	 */
	class Substitution {
	public:
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
	};

	/**
	 */
	class QueryResult {
	public:
		static const QueryResult& noSolution();
		
		QueryResult(const std::shared_ptr<Substitution> &substitution);
		
		bool hasSolution()
		{ return hasSolution_; }
		
		const std::shared_ptr<Substitution>& substitution()
		{ return substitution_; }

	private:
		std::shared_ptr<Substitution> substitution_;
		bool hasSolution_;
		
		// private constructor for "noSolution" QueryResult
		QueryResult();
	};
	
	/**
	 */
	using QueryResultQueue = std::vector<QueryResult>;
}

#endif //__KNOWROB_QUERIES_H__
