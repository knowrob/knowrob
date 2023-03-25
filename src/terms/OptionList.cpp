/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/OptionList.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/terms/Constant.h"

using namespace knowrob;

OptionList::OptionList(const TermPtr &t)
{
	if(t->type() == TermType::LIST) {
		auto *list = (ListTerm*)t.get();
		for(const auto &option : (*list)) {
			readOption(option);
		}
	}
	else {
		readOption(t);
	}
}

void OptionList::readOption(const TermPtr &option) {
	if(option->type() != TermType::PREDICATE) return;
	auto *p = (Predicate*)option.get();

	if(p->indicator()->arity()==2) {
		// an option of the form `Key = Value`
		if (p->indicator()->functor() != "=") return;
		auto keyTerm = p->arguments()[0];
		auto valTerm = p->arguments()[1];
		// TODO: rather allow nullary predicate here?
		if (keyTerm->type() != TermType::STRING) return;
		options_[((StringTerm *) keyTerm.get())->value()] = valTerm;
	}
	else if(p->indicator()->arity()==1) {
		// an option of the form `Key(Value)`
		options_[p->indicator()->functor()] = p->arguments()[0];
	}
}

bool OptionList::contains(const std::string &key) const {
	return options_.count(key)>0;
}

const TermPtr& OptionList::get(const std::string &key, const TermPtr &defaultValue) const {
	auto it = options_.find(key);
	if(it == options_.end()) {
		return defaultValue;
	}
	else {
		return it->second;
	}
}

const std::string& OptionList::getString(const std::string &key, const std::string &defaultValue) const {
	auto it = options_.find(key);
	if(it == options_.end()) {
		return defaultValue;
	}
	else if(it->second->type() == TermType::STRING) {
		return ((StringTerm*)it->second.get())->value();
	}
	else if(it->second->type() == TermType::PREDICATE) {
		Predicate *p = ((Predicate*)it->second.get());
		if(p->indicator()->arity()==0) {
			return p->indicator()->functor();
		}
		else {
			return defaultValue;
		}
	}
	return defaultValue;
}

long OptionList::getLong(const std::string &key, long defaultValue) const {
	auto it = options_.find(key);
	if(it == options_.end()) {
		return defaultValue;
	}
	else if(it->second->type() == TermType::INT32) {
		return ((Integer32Term*)it->second.get())->value();
	}
	else if(it->second->type() == TermType::LONG) {
		return ((LongTerm*)it->second.get())->value();
	}
	return defaultValue;
}
