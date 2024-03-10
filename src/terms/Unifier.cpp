/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/Logger.h"
#include "knowrob/terms/Unifier.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/Numeric.h"

// macro toggles on *occurs* check in unification
#define USE_OCCURS_CHECK

using namespace knowrob;

Unifier::Unifier(const TermPtr &t0, const TermPtr &t1)
		: Bindings(),
		  t0_(t0),
		  t1_(t1),
		  exists_(unify(t0, t1)) {
}

bool Unifier::unify(const TermPtr &t0, const TermPtr &t1) //NOLINT
{
	if (t1->termType() == TermType::VARIABLE) {
		return unify(std::static_pointer_cast<Variable>(t1), t0);
	} else {
		switch (t0->termType()) {
			case TermType::VARIABLE:
				return unify(std::static_pointer_cast<Variable>(t0), t1);
			case TermType::ATOMIC:
				// if one of the terms is an atomic, the other must be an equal atomic
				return *t0 == *t1;
			case TermType::FUNCTION: {
				// n-ary functions with n>0 only unify with other n-ary functions
				if (t1->termType() != TermType::FUNCTION) {
					return false;
				}
				auto *p0 = (Function *) t0.get();
				auto *p1 = (Function *) t1.get();
				// tests for functor equality, and matching arity
				if (*p0->functor() != *p1->functor() ||
					p0->arity() != p1->arity()) {
					return false;
				}
				// unify all arguments
				for (int i = 0; i < p0->arity(); ++i) {
					if (!unify(p0->arguments()[i], p1->arguments()[i])) {
						return false;
					}
				}
				break;
			}
		}
	}

	return true;
}

bool Unifier::unify(const std::shared_ptr<Variable> &var, const TermPtr &t) {
#ifdef USE_OCCURS_CHECK
	if (t->variables().find(var->name()) != t->variables().end()) {
		// fail if var *occurs* in t (occurs check)
		return false;
	} else {
#endif
		set(var, t);
		return true;
#ifdef USE_OCCURS_CHECK
	}
#endif
}

TermPtr Unifier::apply() {
	if (!exists_) {
		// no unifier exists
		return Bottom::get()->functor();
	} else if (t0_->isGround() || t1_->termType() == TermType::VARIABLE) {
		// empty unifier, or only substitutions in t1
		return t0_;
	} else if (t1_->isGround() || t0_->termType() == TermType::VARIABLE) {
		// only substitutions in t0
		return t1_;
	} else if (t0_->termType() == TermType::FUNCTION) {
		// both t0_ and t1_ contain variables.
		auto &p = (
				t0_->variables().size() < t1_->variables().size() ?
				t0_ : t1_);
		return applyBindings(p, *this);
	} else {
		KB_WARN("something went wrong.");
		return Bottom::get()->functor();
	}
}

TEST(unifier, unify) {
	auto varX = std::make_shared<Variable>("X");
	auto varX_2 = std::make_shared<Variable>("X");
	auto varY = std::make_shared<Variable>("Y");

	auto x0 = std::make_shared<Function>("p", std::vector<TermPtr>{varX});
	auto x1 = std::make_shared<Function>("p", std::vector<TermPtr>{varY});
	auto x2 = std::make_shared<Function>("q", std::vector<TermPtr>{varX});
	auto x3 = std::make_shared<Function>("p", std::vector<TermPtr>{varX, varY});
	auto x4 = std::make_shared<Function>("p", std::vector<TermPtr>{
			std::make_shared<Function>("p", std::vector<TermPtr>{varX_2})});
	auto x5 = std::make_shared<Function>("p", std::vector<TermPtr>{
			std::make_shared<Long>(4)});

	// some positive examples:
	// - variable aliasing
	EXPECT_TRUE(Unifier(x0, x1).exists());
	// - instantiation of a variable to a constant
	EXPECT_TRUE(Unifier(x0, x5).exists());
#ifndef USE_OCCURS_CHECK
	// - instantiation of a variable to a term in which the variable occurs (without occurs check)
	EXPECT_TRUE(Unifier(x0,x4).exists());
	// for positive examples, the unifier can be applied to get an instance of the input terms
	EXPECT_EQ(*((Term*)x4.get()), *(Unifier(x0,x4).apply()));
#endif
	EXPECT_EQ(*((Term *) x5.get()), *(Unifier(x0, x5).apply()));

	// some negative examples
	// - functor missmatch
	EXPECT_FALSE(Unifier(x0, x2).exists());
	EXPECT_FALSE(Unifier(x1, x2).exists());
	// - arity missmatch
	EXPECT_FALSE(Unifier(x0, x3).exists());
	EXPECT_FALSE(Unifier(x1, x3).exists());
	EXPECT_FALSE(Unifier(x2, x3).exists());
#ifdef USE_OCCURS_CHECK
	// - occurs check
	EXPECT_FALSE(Unifier(x0, x4).exists());
#endif
}
