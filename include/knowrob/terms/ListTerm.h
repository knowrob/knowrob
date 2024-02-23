/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_LIST_TERM_H_
#define KNOWROB_LIST_TERM_H_

#include <vector>
#include <memory>
#include <ostream>
#include "Function.h"

namespace knowrob {
	/**
	 * A Function representing a list of terms, the functor '[]' is used.
	 * The empty list is a special constant NIL represented as null-ary '[]'.
	 */
	class ListTerm : public Function {
	public:
		explicit ListTerm(const std::vector<TermPtr> &elements);

		static const AtomPtr &listFunctor();

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
		auto &elements() const { return arguments_; }

		/**
		 * @return an iterator ovr the elements of this list.
		 */
		auto begin() { return arguments_.begin(); }

		/**
		 * @return the iterator object indicating the end of iteration.
		 */
		auto end() { return arguments_.end(); }

		// Override Term
		void write(std::ostream &os) const override;
	};
}

#endif //KNOWROB_LIST_TERM_H_
