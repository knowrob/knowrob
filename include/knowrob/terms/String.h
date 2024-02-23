/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_STRING_H
#define KNOWROB_STRING_H

#include "XSDAtomic.h"

namespace knowrob {
	/**
	 * Base class for string terms.
	 */
	class StringBase : public XSDAtomic {
	public:
		StringBase() = default;

		/**
		 * @param other another string
		 * @return true if both strings are equal
		 */
		bool isSameString(const StringBase &other) const { return stringForm() == other.stringForm(); }

		// override Atomic
		AtomicType atomicType() const final { return AtomicType::STRING; }

		// override XSDAtomic
		XSDType xsdType() const final { return XSDType::STRING; }
	};

	/**
	 * A template class for string terms which stores a string of type StrType.
	 * @tparam StrType the type of the string
	 */
	template<typename StrType>
	class StringTemplate : public StringBase {
	public:
		explicit StringTemplate(std::string_view str) : str_(str) {}

		// override Atomic
		std::string_view stringForm() const override { return str_; }

	protected:
		StrType str_;

		// override Term
		void write(std::ostream &os) const override { os << '"' << str_ << '"'; }
	};

	/**
	 * A string term that holds a copy of a string.
	 */
	using String = StringTemplate<std::string>;
	/**
	 * A string term that holds a view on a string which is allocated elsewhere.
	 * @note Make sure the string is not deallocated before the string term is destroyed.
	 */
	using StringView = StringTemplate<std::string_view>;

} // knowrob

#endif //KNOWROB_STRING_H
