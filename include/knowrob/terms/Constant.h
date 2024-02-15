/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CONSTANT_H_
#define KNOWROB_CONSTANT_H_

#include <ostream>
#include <utility>
#include "Term.h"
#include "knowrob/formulas/Predicate.h"

namespace knowrob {
	/**
	 * A typed constant.
	 */
	template <typename T> class Constant : public Term {
	public:
		/**
		 * @type the type of this term.
		 * @value the value.
		 */
		Constant(TermType type, T value)
		: Term(type), value_(std::move(value)) {}

		// Override '<' operator
		bool operator<(const Constant<T> &other) const { return value_ < other.value_; }
		
		/**
		 * @return the typed data value.
		 */
		const T& value() const { return value_; }
		
		// Override Term
		bool isGround() const override { return true; }
		
		// Override Term
		bool isAtomic() const override { return true; }

		// Override Term
		const VariableSet& getVariables() override { return Term::noVariables_; }
		
		// Override Term
		void write(std::ostream& os) const override { os << value_; }

		// Override Term
        size_t computeHash() const override { return std::hash<T>{}(value_); }
	
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
		explicit StringTerm(const std::string &v)
				: Constant(TermType::STRING, v) {}
		explicit StringTerm(const char *v)
				: Constant(TermType::STRING, v) {}

		template<typename ... Args> PredicatePtr operator()(Args&& ... args) const {
			return std::make_shared<Predicate>(value_, readArgs(std::forward<Args>(args)...));
		}
		
		// Override Term
		void write(std::ostream& os) const override;
    protected:
        static void write1(std::ostream& os, const std::string &str);

		template<typename ... Args> std::vector<TermPtr> readArgs(Args&& ... args) const {
			// use a fold expression to convert the arguments to a vector
			// @see https://en.cppreference.com/w/cpp/language/fold
			std::vector<TermPtr> argTerms;
			([&args,&argTerms] { argTerms.push_back(args); } (), ...);
			return argTerms;
		}
	};
	
	/**
	 * A floating point value.
	 */
	class DoubleTerm : public Constant<double> {
	public:
		explicit DoubleTerm(const double &v)
				: Constant(TermType::DOUBLE, v) {}
	};
	
	/**
	 * A long value.
	 */
	class LongTerm : public Constant<long> {
	public:
		explicit LongTerm(const long &v)
				: Constant(TermType::LONG, v) {}
	};
	
	/**
	 * An integer with 32 bit encoding.
	 */
	class Integer32Term : public Constant<int32_t> {
	public:
		explicit Integer32Term(const int32_t &v)
				: Constant(TermType::INT32, v) {}
	};
}

#endif //KNOWROB_CONSTANT_H_
