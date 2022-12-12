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
#include <set>
#include <queue>
#include <memory>
#include <iostream>

namespace knowrob {
	/** The type of a term.
	 */
	enum class TermType {
		PREDICATE,
		VARIABLE,
		STRING,
		DOUBLE,
		INT32,
		LONG,
		TOP,
		BOTTOM
	};
	
	/** An expression in the querying language.
	 * Terms are used as components of formulas and are recursively
	 * constructed over the set of constants, variables, function symbols,
	 * and predicate symbols.
	 * Note that all terms are immutable.
	 */
	class Term {
	public:
		/**
		 * @type the type of this term.
		 */
		Term(TermType type);
		
		/**
		 * @return the type of this term.
		 */
		const TermType& type() const { return type_; }
		
		/**
		 * @return true if this is the top concept.
		 */
		bool isTop() const;
		
		/**
		 * @return true if this is the bottom concept.
		 */
		bool isBottom() const;
		
		/**
		 * @return true if this term contains no variables.
		 */
		virtual bool isGround() const = 0;
		
		/**
		 * @return true if this term is bound and not compound.
		 */
		virtual bool isAtomic() const = 0;
		
		/** Write the term into an ostream.
		 */
		virtual void write(std::ostream& os) const = 0;
	
	private:
		const TermType type_;
	};
	
	/** A term with a fixed truth value being `true`.
	 */
	class TopTerm : public Term {
	public:
		static const std::shared_ptr<TopTerm>& get();
		
		// Override Term
		bool isGround() const { return true; }
		
		// Override Term
		bool isAtomic() const { return true; }
		
		// Overload Term
		void write(std::ostream& os) const;
	
	private:
		TopTerm() : Term(TermType::TOP) {};
	};
	
	/** A term with a fixed truth value being `false`.
	 */
	class BottomTerm : public Term {
	public:
		static const std::shared_ptr<BottomTerm>& get();
		
		// Override Term
		bool isGround() const { return true; }
		
		// Override Term
		bool isAtomic() const { return true; }
		
		// Overload Term
		void write(std::ostream& os) const;
	
	private:
		BottomTerm() : Term(TermType::BOTTOM) {};
	};
	
	/** A variable term.
	 * It is identified by a name string in the scope of a formula,
	 * i.e. within a formula two variables with the same name are considered to be equal.
	 */
	class Variable : public Term {
	public:
		/**
		 * @name the name of the variable.
		 */
		Variable(const std::string &name);
		
		/**
		 * @return the name of this variable.
		 */
		const std::string& name() const { return name_; }
		
		// Override '<' operator needed for using a Variable as a key in std::map
		bool operator< (const Variable& other) const;
		
		// Override Term
		bool isGround() const { return false; }
		
		// Override Term
		bool isAtomic() const { return false; }
		
		// Overload Term
		void write(std::ostream& os) const;

	protected:
		const std::string name_;
	};

	/** A typed constant.
	 */
	template <typename T> class Constant : public Term {
	public:
		/**
		 * @type the type of this term.
		 * @value the value.
		 */
		Constant(TermType type, const T &value)
		: Term(type), value_(value) {}
		
		// Override '<' operator
		bool operator< (const Constant<T>& other) const { return value_ < other.value_; }
		
		/**
		 * @return the typed data value.
		 */
		const T& value() { return value_; }
		
		// Override Term
		bool isGround() const { return true; }
		
		// Override Term
		bool isAtomic() const { return true; }
		
		// Override Term
		void write(std::ostream& os) const { os << value_; }
	
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
		PredicateIndicator(const std::string &functor, unsigned int arity);
		
		// Override '<' operator
		bool operator< (const PredicateIndicator& other) const;
		
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
		
		// Override Term
		void write(std::ostream& os) const;
	
	private:
		const std::string functor_;
		const unsigned int arity_;
	};
	
	// forward declaration
	class Substitution;

	/** A predicate with a functor and a number of term arguments.
	 */
	class Predicate : public Term {
	public:
		/**
		 * @functor the functor name.
		 * @arguments vector of predicate arguments.
		 */
		Predicate(
			const std::string &functor,
			const std::vector<std::shared_ptr<Term>> &arguments);
		
		/**
		 * @indicator a predicate indicator reference.
		 * @arguments vector of predicate arguments.
		 */
		Predicate(
			const std::shared_ptr<PredicateIndicator> &indicator,
			const std::vector<std::shared_ptr<Term>> &arguments);
		
		/** Substitution constructor.
		 *
		 * @other a predicate.
		 * @sub a mapping from terms to variables.
		 */
		Predicate(const Predicate &other, const Substitution &sub);

		/** Get the indicator of this predicate.
		 * @return the indicator of this predicate.
		 */
		const PredicateIndicator& indicator() const { return *indicator_.get(); }

		/** Get the arguments of this predicate.
		 * @return a vector of predicate arguments.
		 */
		const std::vector<std::shared_ptr<Term>>& arguments() const { return arguments_; }
		
		/** Create a copy of this predicate where variables are replaced by terms.
		 * @sub a mapping from variables to terms.
		 */
		std::shared_ptr<Predicate> applySubstitution(const Substitution &sub) const;
		
		// Override Term
		bool isGround() const { return isGround_; }
		
		// Override Term
		bool isAtomic() const { return false; }
		
		// Override Term
		void write(std::ostream& os) const;
	
	protected:
		const std::shared_ptr<PredicateIndicator> indicator_;
		const std::vector<std::shared_ptr<Term>> arguments_;
		const bool isGround_;
		
		bool isGround1() const;
		
		std::vector<std::shared_ptr<Term>> applySubstitution(
			const std::vector<std::shared_ptr<Term>> &in,
			const Substitution &sub) const;
	};
	
	/*
	class Triple : public Predicate {
	};
	
	class FuzzyPredicate : public Predicate {
	};
	
	class TemporalizedPredicate : public Predicate {
	};
	*/
	
	/** A substitution is a mapping from variables to terms.
	 * For example, {x1 -> t1, ..., xn -> tn} represents a substitution of
	 * each variable xi with the corresponding term ti.
	 * Applying a substitution to a term t means to replace occurrences
	 * of each xi with ti. The resulting term is referred to as an *instance* of t.
	 */
	class Substitution {
	public:
		// forward declaration
		class Operation;
		// alias
		using Diff = std::queue<std::shared_ptr<Substitution::Operation>>;
		using Iterator = std::map<Variable, std::shared_ptr<Term>>::iterator;
		
		/**
		 * @var a variable.
		 * @term a term.
		 */
		void set(const Variable &var, const std::shared_ptr<Term> &term);
		
		/** Map a variable to a term.
		 * A null pointer reference is returned if the given variable
		 * is not included in the mapping.
		 *
		 * @var a variable.
		 * @return a term reference.
		 */
		std::shared_ptr<Term> get(const Variable &var) const;
		
		/** Returns true if the given var is mapped to a term by this substitution.
		 * @var a variable.
		 * @return true if this substitution contains the variable.
		 */
		bool contains(const Variable &var) const;
		
		/** Combine with another substitution.
		 * If both substitute the same variable to some term, then
		 * the combination maps to the unification of these terms,
		 * if one exists.
		 * @other another substitution
		 * @changes the diff of the substitute operation
		 * @return true if the operation succeeded.
		 */
		bool combine(const std::shared_ptr<Substitution> &other, Substitution::Diff &changes);
		
		/** Reverts changes made to a substitution.
		 * @changes the diff of a substitute operation
		 */
		void rollBack(Substitution::Diff &changes);
		
		/** An atomic operation performed on a substitution.
		 */
		class Operation {
		public:
			/** Roll back this operation.
			 * @sub a substitution.
			 */
			virtual void rollBack(Substitution &sub) = 0;
		};

	protected:
		std::map<Variable, std::shared_ptr<Term>> mapping_;
		
		class Added : public Operation {
		public:
			Added(const Substitution::Iterator &it);
			// Overwrite Operation
			void rollBack(Substitution &sub);
		protected:
			Substitution::Iterator it_;
		};
		
		class Replaced : public Operation {
		public:
			Replaced(const Substitution::Iterator &it, const std::shared_ptr<Term> &replacedInstance);
			// Overwrite Operation
			void rollBack(Substitution &sub);
		protected:
			Substitution::Iterator it_;
			const std::shared_ptr<Term> replacedInstance_;
		};
	};
	
	// alias declaration
	using SubstitutionPtr = std::shared_ptr<Substitution>;
	
	/** A substitution that unifies some terms.
	 */
	class Unifier : public Substitution {
	public:
		/** Compute a unifier of two terms.
		 * @t0 a term.
		 * @t1 a term.
		 */
		Unifier(const std::shared_ptr<Term> &t0, const std::shared_ptr<Term> &t1);
		
		/**
		 * @return true is a unifier exists.
		 */
		bool exists() const { return exists_; }
		
		/** Applies the unifier to one of the unified terms.
		 * @return an instance of the unified terms.
		 */
		std::shared_ptr<Term> apply();
	
	protected:
		std::shared_ptr<Term> t0_;
		std::shared_ptr<Term> t1_;
		bool exists_;
		
		bool unify(const std::shared_ptr<Term> &t0, const std::shared_ptr<Term> &t1);
		bool unify(const Variable &var, const std::shared_ptr<Term> &t);
	};
}

#endif //__KNOWROB_TERMS_H__
