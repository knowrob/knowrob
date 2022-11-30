/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/lang/terms.h>

using namespace knowrob;

std::list<Variable*> Term::getVariables() {
	std::list<Variable*> l;
	getVariables(l);
	return l;
}

void Variable::getVariables(std::list<Variable*> &list)
{
	list.push_back(this);
}

void Predicate::getVariables(std::list<Variable*> &list)
{
	for(std::shared_ptr<Term> &arg : arguments_) {
		arg->getVariables(list);
	}
}

