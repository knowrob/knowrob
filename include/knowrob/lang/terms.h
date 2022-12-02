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
		Term(TermType type)
		: type_(type) {}
		
		const TermType& type() const { return type_; }
	private:
		TermType type_;
	};
	
	/** A variable term.
	 * Variables may appear free or bound in formulae.
	 * A variable are identified by a name string in the scope of a formula,
	 * i.e. within a formula two variables with the same name are considered to be equal.
	 */
	class Variable : public Term {
	public:
		/** Default constructor.
		 */
		Variable(const std::string &name)
		: Term(TermType::VARIABLE), name_(name) {}
		
		// Override '<' needed for using a Variable as a key in std::map
		bool operator< (const Variable& other) const
		{ return (other.name_ < this->name_); }
		
		/** Get the name of this variable.
		 */
		const std::string& name() { return name_; }

	protected:
		std::string name_;
	};

	/** A typed data value.
	 */
	template <typename T> class Constant : public Term {
	public:
		/** Default constructor.
		 */
		Constant(TermType type, const T &value)
		: Term(type), value_(value) {}
		
		/** Get the typed data value.
		 * @return the value.
		 */
		const T& value() { return value_; }
	
	protected:
		T value_;
	};
	
	/**
	 */
	class StringAtom : public Constant<std::string> {
	public:
		StringAtom(const std::string &v)
		: Constant(TermType::STRING, v) {}
	};
	
	/**
	 */
	class DoubleAtom : public Constant<double> {
	public:
		DoubleAtom(const double &v)
		: Constant(TermType::DOUBLE, v) {}
	};
	
	/**
	 */
	class LongAtom : public Constant<long> {
	public:
		LongAtom(const long &v)
		: Constant(TermType::LONG, v) {}
	};
	
	/**
	 */
	class Integer32Atom : public Constant<int32_t> {
	public:
		Integer32Atom(const int32_t &v)
		: Constant(TermType::INT32, v) {}
	};

	/** The indicator of a predicate defined by its functor and arity.
	 */
	class PredicateIndicator {
	public:
		/** Default constructor.
		 */
		PredicateIndicator(const std::string &functor, unsigned int arity)
		: functor_(functor), arity_(arity) {};
		
		// Override '<' needed for using a PredicateIndicator as a key in std::map
		bool operator< (const PredicateIndicator& other) const
		{ return (other.functor_ < this->functor_) ||
		         (other.arity_   < this->arity_); }
		
		/** Get the functor of this predicate.
		 *
		 * @return functor name
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
		/** Default constructor.
		 */
		Predicate(const std::string &functor, const std::vector<std::shared_ptr<Term>> &arguments)
		: Term(TermType::PREDICATE),
		  indicator_(functor, arguments.size()),
		  arguments_(arguments)
		{}

		/** Get the indicator of this predicate.
		 * @return the indicator of this predicate.
		 */
		const PredicateIndicator& indicator() const { return indicator_; }

		/** Get the arguments of this predicate.
		 * @return a vector of predicate arguments.
		 */
		const std::vector<std::shared_ptr<Term>>& arguments() const { return arguments_; }
	
	protected:
		PredicateIndicator indicator_;
		std::vector<std::shared_ptr<Term>> arguments_;
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
