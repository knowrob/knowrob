/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/formulas/AtomicProposition.h>

using namespace knowrob;

AtomicProposition::AtomicProposition(const std::shared_ptr<Predicate> &predicate)
: Formula(FormulaType::PREDICATE),
  predicate_(predicate)
{
}

bool AtomicProposition::isGround() const
{
	return predicate_->isGround();
}

std::shared_ptr<Formula> AtomicProposition::applySubstitution(const Substitution &sub) const
{
	return std::shared_ptr<Formula>(new AtomicProposition(
		predicate_->applySubstitution(sub)));
}

void AtomicProposition::write(std::ostream& os) const
{
	predicate_->write(os);
}
