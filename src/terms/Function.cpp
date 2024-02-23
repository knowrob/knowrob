/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/terms/Function.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

Function::Function(AtomPtr functor, const std::vector<TermPtr> &arguments)
		: Term(),
		  functor_(std::move(functor)),
		  arguments_(arguments),
		  variables_(getVariables1()) {
}

Function::Function(std::string_view functor, const std::vector<TermPtr> &arguments)
		: Function(Atom::Tabled(functor), arguments) {
}

std::set<std::string_view> Function::getVariables1() const {
	std::set<std::string_view> out;
	for (auto &arg: arguments_) {
		out.insert(arg->variables().begin(), arg->variables().end());
	}
	return out;
}

bool Function::isSameFunction(const Function &other) const {
	if (arguments_.size() != other.arguments_.size() || *functor_ != *other.functor_) {
		return false;
	}
	for (uint32_t i = 0; i < arguments_.size(); i++) {
		if (*arguments_[i] != *other.arguments_[i]) {
			return false;
		}
	}
	return true;
}

size_t Function::hash() const {
	auto seed = static_cast<size_t>(0);
	hashCombine(seed, functor_->hash());
	for (auto &arg: arguments_) {
		hashCombine(seed, arg->hash());
	}
	return seed;
}

void Function::write(std::ostream &os) const {
	os << *functor_;
	if (!arguments_.empty()) {
		os << '(';
		for (uint32_t i = 0; i < arguments_.size(); i++) {
			os << *arguments_[i];
			if (i + 1 < arguments_.size()) {
				os << ',' << ' ';
			}
		}
		os << ')';
	}
}
