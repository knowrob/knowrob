/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_TERMS_H__
#define __KNOWROB_TERMS_H__

// STD
#include <list>
#include <vector>
#include <string>
#include <map>
#include <memory>

namespace knowrob {
	/** The type of a formula.
	 */
	enum class TermType {
		PREDICATE,
		VARIABLE,
		STRING,
		DOUBLE,
		INT32,
		LONG
	};
	
	/** An expression in the querying language.
	 */
	class Term {
	public:
		/**
		 * @type the type of this term.
		 */
		Term(TermType type)
		: type_(type) {}
		
		/** Get the term type.
		 *
		 * @return the type of this term.
		 */
		const TermType& type() const { return type_; }
		
		/**
		 * @return true if this term contains free variables.
		 */
		virtual bool hasFreeVariable() = 0;
	private:
		TermType type_;
	};
	
	/** A variable term.
	 * Variables may appear free or bound in formulae.
	 * A variable is identified by a name string in the scope of a formula,
	 * i.e. within a formula two variables with the same name are considered to be equal.
	 */
	class Variable : public Term {
	public:
		/**
		 * @name the name of the variable.
		 */
		Variable(const std::string &name)
		: Term(TermType::VARIABLE), name_(name) {}
		
		// Override '<' needed for using a Variable as a key in std::map
		bool operator< (const Variable& other) const
		{ return (other.name_ < this->name_); }
		
		/** Get the name of this variable.
		 */
		const std::string& name() { return name_; }
		
		// Override Term
		bool hasFreeVariable() { return true; }

	protected:
		std::string name_;
	};
	
	/** A substitution is a mapping from variables to terms.
	 * e.g. {x1 -> t1, ..., xn -> tn} represents a substitution of
	 * each variable xi with the corresponding term ti.
	 * Applying a substitution to a term t means to replace occurrences
	 * of each xi with ti. The resulting term is referred to as an *instance* of t.
	 */
	class Substitution {
	public:
		/** In FOL, a substitution is seen as a mapping from variables to terms.
		 * It is often denoted as \sigma
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
		
		/** Returns true if the given var is mapped to a term by this substitution.
		 * @var a variable.
		 * @return true if this substitution contains the variable.
		 */
		bool contains(const Variable &var) const;

	private:
		std::map<Variable, std::shared_ptr<Term>> mapping_;
	};
	
	// alias declaration
	using SubstitutionPtr = std::shared_ptr<Substitution>;

	/** A typed data value.
	 */
	template <typename T> class Constant : public Term {
	public:
		/**
		 * @type the type of this term.
		 * @value the value.
		 */
		Constant(TermType type, const T &value)
		: Term(type), value_(value) {}
		
		/** Get the typed data value.
		 * @return the value.
		 */
		const T& value() { return value_; }
		
		// Override Term
		bool hasFreeVariable() { return false; }
	
	protected:
		const T value_;
	};
	
	/** A string value.
	 */
	class StringTerm : public Constant<std::string> {
	public:
		StringTerm(const std::string &v);
	};
	
	/** A floating point value.
	 */
	class DoubleTerm : public Constant<double> {
	public:
		DoubleTerm(const double &v);
	};
	
	/** A long value.
	 */
	class LongTerm : public Constant<long> {
	public:
		LongTerm(const long &v);
	};
	
	/** An integer with 32 bit encoding.
	 */
	class Integer32Term : public Constant<int32_t> {
	public:
		Integer32Term(const int32_t &v);
	};

	/** The indicator of a predicate defined by its functor and arity.
	 */
	class PredicateIndicator {
	public:
		/**
		 * @functor the functor name.
		 * @arity thr arity of this predicate.
		 */
		PredicateIndicator(const std::string &functor, unsigned int arity)
		: functor_(functor), arity_(arity) {};
		
		// Override '<' needed for using a PredicateIndicator as a key in std::map
		bool operator< (const PredicateIndicator& other) const
		{ return (other.functor_ < this->functor_) ||
		         (other.arity_   < this->arity_); }
		
		/** Get the functor of this predicate.
		 *
		 * @return the functor name.
		 */
		const std::string& functor() const { return functor_; }
		
		/** Get the arity of this predicate.
		 *
		 * @return arity of predicate
		 */
		unsigned int arity() const { return arity_; }
	private:
		std::string functor_;
		unsigned int arity_;
	};

	/** A predicate with a functor and a number of term arguments.
	 */
	class Predicate : public Term {
	public:
		/**
		 * @functor the functor name.
		 * @arguments list of predicate arguments.
		 */
		Predicate(const std::string &functor, const std::vector<std::shared_ptr<Term>> &arguments);

		/** Get the indicator of this predicate.
		 * @return the indicator of this predicate.
		 */
		const PredicateIndicator& indicator() const { return indicator_; }

		/** Get the arguments of this predicate.
		 * @return a vector of predicate arguments.
		 */
		const std::vector<std::shared_ptr<Term>>& arguments() const { return arguments_; }
		
		/**
		 */
		void applySubstitution(const Substitution &sub);
		
		// Override Term
		bool hasFreeVariable() { return hasFreeVariable_; }
	
	protected:
		PredicateIndicator indicator_;
		std::vector<std::shared_ptr<Term>> arguments_;
		bool hasFreeVariable_;
	};
	
	/*
	class Triple : public Predicate {
	};
	
	class FuzzyPredicate : public Predicate {
	};
	
	class TemporalizedPredicate : public Predicate {
	};
	*/
}

#endif //__KNOWROB_TERMS_H__
