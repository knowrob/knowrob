/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_LIST_TERM_H_
#define KNOWROB_LIST_TERM_H_

#include <vector>
#include <memory>
#include <ostream>
#include "Term.h"

namespace knowrob {
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

		// Override Term
        size_t computeHash() const override;
	
	protected:
		const std::vector<TermPtr> elements_;
		const VariableSet variables_;

		VariableSet getVariables1() const;
		// Override Term
		bool isEqual(const Term &other) const override;
	};
}

#endif //KNOWROB_LIST_TERM_H_
