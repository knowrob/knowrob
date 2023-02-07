/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include "knowrob/logging.h"
#include "knowrob/terms/Unifier.h"
#include "knowrob/terms/Predicate.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/Bottom.h"

// macro toggles on *occurs* check in unification
#define USE_OCCURS_CHECK

using namespace knowrob;

Unifier::Unifier(const TermPtr &t0, const TermPtr &t1)
: Substitution(),
  t0_(t0),
  t1_(t1),
  exists_(unify(t0,t1))
{
}

bool Unifier::unify(const TermPtr &t0, const TermPtr &t1) //NOLINT
{
	if(t1->type() == TermType::VARIABLE) {
		return unify((Variable*)t1.get(), t0);
	}
	else {
		switch(t0->type()) {
		case TermType::VARIABLE:
			return unify((Variable*)t0.get(), t1);
		case TermType::PREDICATE: {
			// predicates only unify with other predicates
			if(t1->type()!=TermType::PREDICATE) {
				return false;
			}
			auto *p0 = (Predicate*)t0.get();
            auto *p1 = (Predicate*)t1.get();
			// tests for functor equality, and matching arity
			if(p0->indicator()->functor() != p1->indicator()->functor() ||
			   p0->indicator()->arity()   != p1->indicator()->arity()) {
				return false;
			}
			// unify all arguments
			for(int i=0; i<p0->indicator()->arity(); ++i) {
				if(!unify(p0->arguments()[i], p1->arguments()[i])) {
					return false;
				}
			}
			break;
		}
		case TermType::STRING:
			return t1->type()==TermType::STRING &&
				((StringTerm*)t0.get())->value()==((StringTerm*)t1.get())->value();
		case TermType::DOUBLE:
			return t1->type()==TermType::DOUBLE &&
				((DoubleTerm*)t0.get())->value()==((DoubleTerm*)t1.get())->value();
		case TermType::INT32:
			return t1->type()==TermType::INT32 &&
				((Integer32Term*)t0.get())->value()==((Integer32Term*)t1.get())->value();
		case TermType::LONG:
			return t1->type()==TermType::LONG &&
				((LongTerm*)t0.get())->value()==((LongTerm*)t1.get())->value();
		default:
			KB_WARN("Ignoring unknown term type '{}'.", (int)t0->type());
			return false;
		}
	}
	
	return true;
}

bool Unifier::unify(const Variable *var, const TermPtr &t)
{
#ifdef USE_OCCURS_CHECK
	if(t->getVariables().find(var) != t->getVariables().end()) {
		// fail if var *occurs* in t (occurs check)
		return false;
	} else {
#endif
		set(*var, t);
		return true;
#ifdef USE_OCCURS_CHECK
	}
#endif
}

TermPtr Unifier::apply()
{
	if(!exists_) {
		// no unifier exists
		return BottomTerm::get();
	}
	else if(mapping_.empty() ||
		t0_->isGround() ||
		t1_->type()==TermType::VARIABLE)
	{
		// empty unifier, or only substitutions in t1
		return t0_;
	}
	else if(t1_->isGround() ||
		t0_->type()==TermType::VARIABLE)
	{
		// only substitutions in t0
		return t1_;
	}
	else if(t0_->type()==TermType::PREDICATE) {
		// both t0_ and t1_ contain variables, so they are either Variables
		// or Predicates where an argument contains a variable.
		// the variable case is covered above so both must be predicates.
		auto *p = (Predicate*)(
				t0_->getVariables().size() < t1_->getVariables().size()?
				t0_:t1_).get();
		return p->applySubstitution(*this);
	}
	else {
		KB_WARN("something went wrong.");
		return BottomTerm::get();
	}
}

TEST(unifier, unify) {
    auto varX = std::make_shared<Variable>("X");
	auto varX_2 = std::make_shared<Variable>("X");
    auto varY = std::make_shared<Variable>("Y");

    auto x0 = std::make_shared<Predicate>("p", std::vector<TermPtr>{varX});
    auto x1 = std::make_shared<Predicate>("p", std::vector<TermPtr>{varY});
    auto x2 = std::make_shared<Predicate>("q", std::vector<TermPtr>{varX});
    auto x3 = std::make_shared<Predicate>("p", std::vector<TermPtr>{varX,varY});
    auto x4 = std::make_shared<Predicate>("p", std::vector<TermPtr>{
        std::make_shared<Predicate>("p", std::vector<TermPtr>{varX_2}) });
    auto x5 = std::make_shared<Predicate>("p", std::vector<TermPtr>{
        std::make_shared<LongTerm>(4) });

    // some positive examples:
    // - variable aliasing
    EXPECT_TRUE(Unifier(x0,x1).exists());
    // - instantiation of a variable to a constant
    EXPECT_TRUE(Unifier(x0,x5).exists());
#ifndef USE_OCCURS_CHECK
    // - instantiation of a variable to a term in which the variable occurs (without occurs check)
    EXPECT_TRUE(Unifier(x0,x4).exists());
    // for positive examples, the unifier can be applied to get an instance of the input terms
    EXPECT_EQ(*((Term*)x4.get()), *(Unifier(x0,x4).apply()));
#endif
    EXPECT_EQ(*((Term*)x5.get()), *(Unifier(x0,x5).apply()));

    // some negative examples
    // - functor missmatch
    EXPECT_FALSE(Unifier(x0,x2).exists());
    EXPECT_FALSE(Unifier(x1,x2).exists());
    // - arity missmatch
    EXPECT_FALSE(Unifier(x0,x3).exists());
    EXPECT_FALSE(Unifier(x1,x3).exists());
    EXPECT_FALSE(Unifier(x2,x3).exists());
#ifdef USE_OCCURS_CHECK
	// - occurs check
	EXPECT_FALSE(Unifier(x0,x4).exists());
#endif
}
