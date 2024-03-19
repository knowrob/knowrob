/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/OptionList.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/terms/Numeric.h"

using namespace knowrob;

OptionList::OptionList(const TermPtr &t) {
	bool optionsRead = false;
	if (t->isFunction()) {
		auto *fn = (Function *) t.get();
		if (*fn->functor() == *ListTerm::listFunctor()) {
			for (auto &arg: fn->arguments()) {
				readOption(arg);
			}
			optionsRead = true;
		}
	}
	if (!optionsRead) {
		readOption(t);
	}
}

void OptionList::readOption(const TermPtr &option) {
	static const auto a_eq = Atom::Tabled("=");

	if (option->termType() != TermType::FUNCTION) return;
	auto *fn = (Function *) option.get();

	if (fn->arity() == 2 && *fn->functor() == *a_eq) {
		// an option of the form `Key = Value`
		auto keyTerm = fn->arguments()[0];
		auto valTerm = fn->arguments()[1];
		if (keyTerm->isAtom()) {
			auto keyAtom = std::static_pointer_cast<Atom>(keyTerm);
			options_[std::string(keyAtom->stringForm())] = valTerm;
		}
	} else if (fn->arity() == 1) {
		// an option of the form `Key(Value)`
		options_[std::string(fn->functor()->stringForm())] = fn->arguments()[0];
	}
}

bool OptionList::contains(const std::string &key) const {
	return options_.count(key) > 0;
}

const TermPtr &OptionList::get(const std::string &key, const TermPtr &defaultValue) const {
	auto it = options_.find(key);
	if (it == options_.end()) {
		return defaultValue;
	} else {
		return it->second;
	}
}

std::string_view OptionList::getString(const std::string &key, const std::string &defaultValue) const {
	auto it = options_.find(key);
	if (it == options_.end()) {
		return defaultValue;
	} else if (it->second->termType() == TermType::FUNCTION) {
		Function *fn = ((Function *) it->second.get());
		if (fn->arity() == 0) {
			return fn->functor()->stringForm();
		} else {
			return defaultValue;
		}
	} else if (it->second->isAtomic()) {
		return std::static_pointer_cast<Atomic>(it->second)->stringForm();
	}
	return defaultValue;
}

long OptionList::getLong(const std::string &key, long defaultValue) const {
	auto it = options_.find(key);
	if (it == options_.end()) {
		return defaultValue;
	} else if (it->second->isNumeric()) {
		return std::static_pointer_cast<Numeric>(it->second)->asLong();
	}
	return defaultValue;
}

std::optional<double> OptionList::getDouble(const std::string &key) const {
	auto it = options_.find(key);
	if (it == options_.end()) {
		return std::nullopt;
	} else if (it->second->isNumeric()) {
		return std::static_pointer_cast<Numeric>(it->second)->asDouble();
	}
	return std::nullopt;
}
