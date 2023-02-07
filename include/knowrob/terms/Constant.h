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
#include "Term.h"

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
		explicit StringTerm(const std::string &v)
				: Constant(TermType::STRING, v) {}
		
		// Override Term
		void write(std::ostream& os) const override
		{ os << '\'' << value_ << '\''; }
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
