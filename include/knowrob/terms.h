/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TERMS_H_
#define KNOWROB_TERMS_H_

// STD
#include <list>
#include <set>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <queue>
#include <memory>
#include <iostream>
#include <functional>

namespace knowrob {
	/**
	 * The type of a term.
	 */
	enum class TermType {
		PREDICATE = 0,
		VARIABLE,
		STRING,
		DOUBLE,
		INT32,
		LONG,
		LIST
	};

	// forward declaration
	class Variable;

	/**
	 * Used to compare variable pointers by value.
	 */
	struct VariableComparator {
		/**
		 * Compares Variable pointers by value.
		 */
		bool operator()(const Variable* const &v0, const Variable* const &v1) const;
	};
	// a set of const variable pointers compared by value.
	using VariableSet = std::set<const Variable*,VariableComparator>;

	/**
	 * An expression in the querying language.
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
		explicit Term(TermType type);

		/**
		 * @param other another term
		 * @return true if both terms are equal
		 */
        bool operator==(const Term& other) const;
		
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
		 * @return true if this term isMoreGeneralThan no variables.
		 */
		virtual bool isGround() const = 0;
		
		/**
		 * @return true if this term is bound and not compound.
		 */
		virtual bool isAtomic() const = 0;

		/**
		 * @return set of variables of this term.
		 */
		virtual const VariableSet& getVariables() = 0;
		
		/**
		 * Write the term into an ostream.
		 */
		virtual void write(std::ostream& os) const = 0;
	
	protected:
		static const VariableSet noVariables_;
		const TermType type_;

		virtual bool isEqual(const Term &other) const = 0;
	};
	
	// alias declaration
	using TermPtr = std::shared_ptr<Term>;
	
	/**
	 * A variable term.
	 * A variable is identified by a name string in the scope of a formula,
	 * i.e. within a formula two variables with the same name are considered to be equal.
	 */
	class Variable : public Term {
	public:
		/**
		 * @name the name of the variable.
		 */
		explicit Variable(std::string name);

		/**
		 * @param other another variable.
		 * @return true if this name is alphabetically before other
		 */
		bool operator< (const Variable& other) const;
		
		/**
		 * @return the name of this variable.
		 */
		const std::string& name() const { return name_; }
		
		// Override Term
		bool isGround() const override { return false; }
		
		// Override Term
		bool isAtomic() const override { return false; }

		// Override Term
		const VariableSet& getVariables() override { return variables_; }
		
		// Override Term
		void write(std::ostream& os) const override;

	protected:
		const std::string name_;
		VariableSet variables_;

		// Override Term
		bool isEqual(const Term &other) const override;
	};

	/**
	 * A typed constant.
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
		bool operator<(const Constant<T> &other) const { return value_ < other.value_; }
		
		/**
		 * @return the typed data value.
		 */
		const T& value() { return value_; }
		
		// Override Term
		bool isGround() const override { return true; }
		
		// Override Term
		bool isAtomic() const override { return true; }

		// Override Term
		const VariableSet& getVariables() override { return Term::noVariables_; }
		
		// Override Term
		void write(std::ostream& os) const override { os << value_; }
	
	protected:
		const T value_;

		// Override Term
		bool isEqual(const Term &other) const override
		{ return value_ == static_cast<const Constant<T>&>(other).value_; }
	};
	
	/**
	 * A string value.
	 */
	class StringTerm : public Constant<std::string> {
	public:
		explicit StringTerm(const std::string &v);
		
		// Override Term
		void write(std::ostream& os) const override;
	};
	
	/**
	 * A floating point value.
	 */
	class DoubleTerm : public Constant<double> {
	public:
		explicit DoubleTerm(const double &v);
	};
	
	/**
	 * A long value.
	 */
	class LongTerm : public Constant<long> {
	public:
		explicit LongTerm(const long &v);
	};
	
	/**
	 * An integer with 32 bit encoding.
	 */
	class Integer32Term : public Constant<int32_t> {
	public:
		explicit Integer32Term(const int32_t &v);
	};

	/**
	 * The indicator of a predicate defined by its functor and arity.
	 */
	class PredicateIndicator {
	public:
		/**
		 * @functor the functor name.
		 * @arity thr arity of this predicate.
		 */
		PredicateIndicator(std::string functor, unsigned int arity);

        // Override '==' operator
        bool operator==(const PredicateIndicator& other) const;
		// Override '<' operator
		bool operator< (const PredicateIndicator& other) const;
		
		/**
		 * Get the functor of this predicate.
		 *
		 * @return the functor name.
		 */
		const std::string& functor() const { return functor_; }
		
		/**
		 * Get the arity of this predicate.
		 *
		 * @return arity of predicate
		 */
		unsigned int arity() const { return arity_; }

		/**
		 * Convert the predicate indicator to a term of the form `'/'(Functor,Arity)`.
		 * @return the indicator as a term.
		 */
		std::shared_ptr<Term> toTerm() const;

		void write(std::ostream& os) const;
	
	private:
		const std::string functor_;
		const unsigned int arity_;
	};

	/**
	 * The type of a predicate.
	 */
	enum class PredicateType {
		BUILT_IN = 0,
		FORMULA,
		RELATION
	};

	/**
	 * Read predicate type from term.
	 * @param term a term.
	 * @return the predicate type encoded by term.
	 */
	PredicateType predicateTypeFromTerm(const TermPtr &term);

	/**
	 * The description of a defined predicate.
	 */
	class PredicateDescription {
	public:
		/**
		 * @param indicator the indicator of the predicate.
		 * @param type the type of the predicate.
		 */
		PredicateDescription(const std::shared_ptr<PredicateIndicator> &indicator, PredicateType type)
		: indicator_(indicator), type_(type) {}

		/**
		 * @return the indicator of the predicate.
		 */
		const std::shared_ptr<PredicateIndicator>& indicator() const { return indicator_; }

		/**
		 * @return the type of the predicate.
		 */
		PredicateType type() const { return type_; }

	protected:
		std::shared_ptr<PredicateIndicator> indicator_;
		PredicateType type_;
	};
	
	// forward declaration
	class Substitution;

	/**
	 * A predicate with a functor and a number of term arguments.
	 */
	class Predicate : public Term {
	public:
		/**
		 * @functor the functor name.
		 * @arguments vector of predicate arguments.
		 */
		Predicate(
			const std::string &functor,
			const std::vector<TermPtr> &arguments);
		
		/**
		 * @indicator a predicate indicator reference.
		 * @arguments vector of predicate arguments.
		 */
		Predicate(
			const std::shared_ptr<PredicateIndicator> &indicator,
			const std::vector<TermPtr> &arguments);
		
		/**
		 * Substitution constructor.
		 *
		 * @other a predicate.
		 * @sub a mapping from terms to variables.
		 */
		Predicate(const Predicate &other, const Substitution &sub);

		/**
		 * Get the indicator of this predicate.
		 * @return the indicator of this predicate.
		 */
		const std::shared_ptr<PredicateIndicator>& indicator() const { return indicator_; }

		/**
		 * Get the arguments of this predicate.
		 * @return a vector of predicate arguments.
		 */
		const std::vector<TermPtr>& arguments() const { return arguments_; }
		
		/**
		 * Create a copy of this predicate where variables are replaced by terms.
		 * @sub a mapping from variables to terms.
		 */
		std::shared_ptr<Predicate> applySubstitution(const Substitution &sub) const;
		
		// Override Term
		bool isGround() const override { return variables_.empty(); }
		
		// Override Term
		bool isAtomic() const override { return false; }

		// Override Term
		const VariableSet& getVariables() override { return variables_; }
		
		// Override Term
		void write(std::ostream& os) const override;
	
	protected:
		const std::shared_ptr<PredicateIndicator> indicator_;
		const std::vector<TermPtr> arguments_;
		const VariableSet variables_;

		VariableSet getVariables1() const;
		// Override Term
		bool isEqual(const Term &other) const override;
		
		static std::vector<TermPtr> applySubstitution(
			const std::vector<TermPtr> &in,
			const Substitution &sub) ;
	};
	
	/**
	 * A predicate with a fixed truth value being `true`.
	 */
	class TopTerm : public Predicate {
	public:
		static const std::shared_ptr<TopTerm>& get();
		
		// Override Term
		void write(std::ostream& os) const override;
	
	private:
		TopTerm();
	};
	
	/**
	 * A predicate with a fixed truth value being `false`.
	 */
	class BottomTerm : public Predicate {
	public:
		static const std::shared_ptr<BottomTerm>& get();
		
		// Override Term
		void write(std::ostream& os) const override;
	
	private:
		BottomTerm();
	};
	
	/**
	 * A composite term that isMoreGeneralThan a list of terms.
	 * The empty list is a special constant NIL.
	 */
	class ListTerm : public Term {
	public:
		explicit ListTerm(const std::vector<TermPtr> &elements);
		
		/**
		 * @return the NIL constant.
		 */
		static std::shared_ptr<ListTerm> nil();
		
		/**
		 * @return true if this list term is the NIL constant.
		 */
		bool isNIL() const;

		/**
		 * Get the elements of this list.
		 * @return a vector of list elements.
		 */
		const std::vector<TermPtr>& elements() const { return elements_; }

		/**
		 * @return an iterator ovr the elements of this list.
		 */
		std::vector<TermPtr>::const_iterator begin() { return elements_.begin(); }

		/**
		 * @return the iterator object indicating the end of iteration.
		 */
		std::vector<TermPtr>::const_iterator end()   { return elements_.end(); }
		
		// Override Term
		bool isGround() const override { return variables_.empty(); }
		
		// Override Term
		bool isAtomic() const override { return isNIL(); }

		// Override Term
		const VariableSet& getVariables() override { return variables_; }
		
		// Override Term
		void write(std::ostream& os) const override;
	
	protected:
		const std::vector<TermPtr> elements_;
		const VariableSet variables_;

		VariableSet getVariables1() const;
		// Override Term
		bool isEqual(const Term &other) const override;
	};

	/**
	 * A list of options, where each option is represented as a term.
	 */
	class OptionList {
	public:
		/**
		 * Constructs an option list from a term.
		 * The term may be a list of options, or a single option value.
		 * Option terms have either the form `Key = Value` or `Key(Value)`.
		 * @param t a term from which options are read.
		 */
		explicit OptionList(const TermPtr &t);

		/**
		 * @return the option map.
		 */
		const std::map<std::string, TermPtr>& options() const { return options_; }

		/**
		 * @param key key of option.
		 * @return true is this list isMoreGeneralThan the key.
		 */
		bool contains(const std::string &key);

		/**
		 * @param key an option key
		 * @param defaultValue a default value
		 * @return the option value, or the default value
		 */
		const TermPtr& get(const std::string &key, const TermPtr &defaultValue);

		/**
		 * Read option value as a string.
		 * @param key an option key
		 * @param defaultValue a default value
		 * @return the option value, or the default value
		 */
		const std::string& getString(const std::string &key, const std::string &defaultValue);

		/**
		 * Read option value as a long.
		 * @param key an option key
		 * @param defaultValue a default value
		 * @return the option value, or the default value
		 */
		long getLong(const std::string &key, long defaultValue);

	protected:
		std::map<std::string, TermPtr> options_;

		void readOption(const TermPtr &option);
	};

	/**
	 * Queue  of reversible operations.
	 */
	class Reversible : public std::queue<std::function<void()>> {
	public:
		/**
		 * Reverts changes made.
		 */
		void rollBack();
	};
	
	/**
	 * A substitution is a mapping from variables to terms.
	 * For example, {x1 -> t1, ..., xn -> tn} represents a substitution of
	 * each variable xi with the corresponding term ti.
	 * Applying a substitution to a term t means to replace occurrences
	 * of each xi with ti. The resulting term is referred to as an *instance* of t.
	 */
	class Substitution {
	public:
		/**
		 * @return true if this substitution does not map a single variable to a term.
		 */
		bool empty() const { return mapping_.empty(); }

		/**
		 * @return begin iterator of substitution.
		 */
		auto begin() const { return mapping_.begin(); }

		/**
		 * @return end iterator of substitution.
		 */
		auto end() const { return mapping_.end(); }
		
		/**
		 * @var a variable.
		 * @term a term.
		 */
		void set(const Variable &var, const TermPtr &term);
		
		/**
		 * Map a variable to a term.
		 * A null pointer reference is returned if the given variable
		 * is not included in the mapping.
		 *
		 * @var a variable.
		 * @return a term reference.
		 */
		const TermPtr& get(const Variable &var) const;
		
		/**
		 * Returns true if the given var is mapped to a term by this substitution.
		 * @var a variable.
		 * @return true if this substitution isMoreGeneralThan the variable.
		 */
		bool contains(const Variable &var) const;

		/**
		 * Combine with another substitution.
		 * If both substitute the same variable to some term, then
		 * the combination maps to the unification of these terms, if one exists.
		 * @other another substitution
		 * @changes the diff of the substitute operation
		 * @return true if the operation succeeded.
		 */
		bool unifyWith(const Substitution &other, Reversible *reversible= nullptr);

	protected:
		std::map<Variable,TermPtr> mapping_;
	};
	
	// alias declaration
	using SubstitutionPtr = std::shared_ptr<Substitution>;
	
	/**
	 * A substitution that unifies some terms.
	 */
	class Unifier : public Substitution {
	public:
		/**
		 * Compute a unifier of two terms.
		 * @t0 a term.
		 * @t1 a term.
		 */
		Unifier(const TermPtr &t0, const TermPtr &t1);
		
		/**
		 * @return true is a unifier exists.
		 */
		bool exists() const { return exists_; }
		
		/**
		 * Applies the unifier to one of the unified terms.
		 * @return an instance of the unified terms.
		 */
		TermPtr apply();
	
	protected:
		TermPtr t0_;
		TermPtr t1_;
		bool exists_;
		
		bool unify(const TermPtr &t0, const TermPtr &t1);
		bool unify(const Variable *var, const TermPtr &t);
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Term& t);
	std::ostream& operator<<(std::ostream& os, const knowrob::Substitution& omega);
}

#endif //KNOWROB_TERMS_H_
