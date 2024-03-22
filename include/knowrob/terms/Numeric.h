/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_NUMERIC_TERM_H
#define KNOWROB_NUMERIC_TERM_H

#include "XSDAtomic.h"
#include "sstream"
#include "optional"
#include "string"

namespace knowrob {
	/**
	 * Base class for numeric terms.
	 */
	class Numeric : public XSDAtomic {
	public:
		Numeric() : XSDAtomic() {}

		/**
		 * @param other another numeric
		 * @return true if both numerics are equal
		 */
		virtual bool isSameNumeric(const Numeric &other) const = 0;

		/**
		 * @return the numeric as a double
		 */
		virtual double asDouble() const = 0;

		/**
		 * @return the numeric as a float
		 */
		virtual float asFloat() const = 0;

		/**
		 * @return the numeric as an integer
		 */
		virtual int asInteger() const = 0;

		/**
		 * @return the numeric as a long
		 */
		virtual long asLong() const = 0;

		/**
		 * @return the numeric as a short
		 */
		virtual short asShort() const = 0;

		/**
		 * @return the numeric as an unsigned integer
		 */
		virtual unsigned int asUnsignedInteger() const = 0;

		/**
		 * @return the numeric as an unsigned long
		 */
		virtual unsigned long asUnsignedLong() const = 0;

		/**
		 * @return the numeric as an unsigned short
		 */
		virtual unsigned short asUnsignedShort() const = 0;

		/**
		 * @return the numeric as a boolean
		 */
		virtual bool asBoolean() const = 0;

		/**
		 * @return a boolean typed numeric with value true
		 */
		static std::shared_ptr<Numeric> trueAtom();

		/**
		 * @return a boolean typed numeric with value false
		 */
		static std::shared_ptr<Numeric> falseAtom();

		/**
		 * @return true if the numeric is a floating number
		 */
		bool isFloatingNumber() const;

		// override Atomic
		AtomicType atomicType() const final { return AtomicType::NUMERIC; }
	};

	/**
	 * A template class for numeric terms which stores a numeric of type T1, and also provides
	 * access to string representation of the numeric.
	 * @tparam T1 the type of the numeric
	 * @tparam T2 the XSD type of the numeric
	 */
	template<typename T1, XSDType T2>
	class NumericTemplate : public Numeric {
	public:
		/**
		 * @param numericForm the numeric form of the term
		 */
		explicit NumericTemplate(T1 numericForm) : Numeric(), numericForm_(numericForm) {}

		/**
		 * @param stringForm the string form of the term
		 */
		explicit NumericTemplate(std::string_view stringForm) : Numeric(), stringForm_(stringForm) {}

		T1 operator()() const { return numericForm(); }

		/**
		 * @return the numeric form of this term.
		 */
		T1 numericForm() const {
			if (!numericForm_) {
				std::istringstream iss(*stringForm_);
				T1 value;
				iss >> std::fixed >> value;
				numericForm_ = value;
			}
			return *numericForm_;
		}

		// override Numeric
		double asDouble() const override { return static_cast<double>(numericForm()); }

		// override Numeric
		float asFloat() const override { return static_cast<float>(numericForm()); }

		// override Numeric
		int asInteger() const override { return static_cast<int>(numericForm()); }

		// override Numeric
		long asLong() const override { return static_cast<long>(numericForm()); }

		// override Numeric
		short asShort() const override { return static_cast<short>(numericForm()); }

		// override Numeric
		unsigned int asUnsignedInteger() const override { return static_cast<unsigned int>(numericForm()); }

		// override Numeric
		unsigned long asUnsignedLong() const override { return static_cast<unsigned long>(numericForm()); }

		// override Numeric
		unsigned short asUnsignedShort() const override { return static_cast<unsigned short>(numericForm()); }

		// override Numeric
		bool asBoolean() const override { return static_cast<bool>(numericForm()); }

		// override Atomic
		std::string_view stringForm() const override {
			if (!stringForm_) {
				std::ostringstream oss;
				oss << std::fixed << std::boolalpha << *numericForm_;
				stringForm_ = oss.str();
			}
			return *stringForm_;
		}

		// override Numeric
		bool isSameNumeric(const Numeric &other) const override {
			if (xsdType() != other.xsdType()) {
				return false;
			}
			const auto otherNumeric = static_cast<const NumericTemplate<T1, T2> *>(&other);
			return numericForm() == otherNumeric->numericForm();
		}

		// Override Term
		size_t hash() const override { return std::hash<T1>()(numericForm()); }

		// override XSDAtomic
		XSDType xsdType() const final { return T2; }

	private:
		// Note: both are mutable because they are initialized in a lazy fashion
		mutable std::optional<std::string> stringForm_;
		mutable std::optional<T1> numericForm_;

		// override Term
		void write(std::ostream &os) const override { os << std::defaultfloat << std::boolalpha << numericForm(); }
	};

	/**
	 * A numeric term that holds a copy of an integer.
	 */
	using Integer = NumericTemplate<int, XSDType::INTEGER>;

	/**
	 * A numeric term that holds a copy of a long.
	 */
	using Long = NumericTemplate<long, XSDType::LONG>;

	/**
	 * A numeric term that holds a copy of a short.
	 */
	using Short = NumericTemplate<short, XSDType::SHORT>;

	/**
	 * A numeric term that holds a copy of an unsigned integer.
	 */
	using UnsignedLong = NumericTemplate<unsigned long, XSDType::UNSIGNED_LONG>;

	/**
	 * A numeric term that holds a copy of an unsigned long.
	 */
	using UnsignedInt = NumericTemplate<unsigned int, XSDType::UNSIGNED_INT>;

	/**
	 * A numeric term that holds a copy of an unsigned short.
	 */
	using UnsignedShort = NumericTemplate<unsigned short, XSDType::UNSIGNED_SHORT>;

	/**
	 * A numeric term that holds a copy of a float.
	 */
	using Float = NumericTemplate<float, XSDType::FLOAT>;

	/**
	 * A numeric term that holds a copy of a double.
	 */
	using Double = NumericTemplate<double, XSDType::DOUBLE>;

	/**
	 * A numeric term that holds a copy of a boolean.
	 */
	using Boolean = NumericTemplate<bool, XSDType::BOOLEAN>;

} // knowrob

#endif //KNOWROB_NUMERIC_TERM_H
