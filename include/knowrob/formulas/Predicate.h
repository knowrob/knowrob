/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PREDICATE_H_
#define KNOWROB_PREDICATE_H_

#include <vector>
#include <memory>
#include <string>
#include "Formula.h"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Bindings.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/Function.h"

namespace knowrob {
	/**
	 * A predicate with a functor and a number of term arguments.
	 */
	class Predicate : public Formula {
	public:
		/**
		 * @functor the functor name.
		 * @arguments vector of predicate arguments.
		 */
		explicit Predicate(std::string_view functor, const std::vector<TermPtr> &arguments = {});

		/**
		 * @functor the functor name.
		 * @arguments vector of predicate arguments.
		 */
		explicit Predicate(AtomPtr functor, const std::vector<TermPtr> &arguments = {});

		/**
		 * Get the functor of this predicate.
		 * @return the functor.
		 */
		auto &functor() const { return functor_; }

		/**
		 * Get the arity of this predicate.
		 * @return the arity.
		 */
		auto arity() const { return arguments().size(); }

		/**
		 * Get the arguments of this predicate.
		 * @return a vector of predicate arguments.
		 */
		const std::vector<TermPtr> &arguments() const { return arguments_; }

		// Override Term, Formula
		bool isGround() const override { return variables_.empty(); }

		// Override Term
		const std::set<std::string_view> &variables() { return variables_; }

		// Override Formula
		void write(std::ostream &os) const override;

		// Override Term
		size_t hash() const;

		static FunctionPtr toFunction(const std::shared_ptr<Predicate> &predicate);

		static std::shared_ptr<Predicate> fromFunction(const FunctionPtr &fn);

	protected:
		const AtomPtr functor_;
		const std::vector<TermPtr> arguments_;
		const std::set<std::string_view> variables_;

		std::set<std::string_view> getVariables1() const;

		bool isEqual(const Formula &other) const override;
	};

	using PredicatePtr = std::shared_ptr<Predicate>;
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Predicate &p);
}

#endif //KNOWROB_PREDICATE_H_
