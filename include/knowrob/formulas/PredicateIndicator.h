/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PREDICATE_INDICATOR_H_
#define KNOWROB_PREDICATE_INDICATOR_H_

#include <vector>
#include <memory>
#include <string>
#include "Formula.h"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Bindings.h"

namespace knowrob {
	/**
	 * The indicator of a predicate defined by its functor and arity.
	 */
	class PredicateIndicator {
	public:
		/**
		 * @functor the functor name.
		 * @arity thr arity of this predicate.
		 */
		PredicateIndicator(std::string_view functor, unsigned int arity);

		// Override '==' operator
		bool operator==(const PredicateIndicator &other) const;

		// Override '<' operator
		bool operator<(const PredicateIndicator &other) const;

		/**
		 * Get the functor of this predicate.
		 *
		 * @return the functor name.
		 */
		const std::string &functor() const { return functor_; }

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

		void write(std::ostream &os) const;

	private:
		const std::string functor_;
		const unsigned int arity_;
	};

	using PredicateIndicatorPtr = std::shared_ptr<PredicateIndicator>;
}

#endif //KNOWROB_PREDICATE_INDICATOR_H_
