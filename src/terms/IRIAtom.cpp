/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/IRIAtom.h"

using namespace knowrob;

PredicatePtr IRIAtom::operator()(const TermPtr &s, const TermPtr &o) const {
	auto functor = IRIAtom::Tabled(stringForm());
	return std::make_shared<Predicate>(functor, std::vector<TermPtr>{s, o});
}

std::shared_ptr<knowrob::IRIAtom> IRIAtom::Tabled(std::string_view name) {
	auto it = Atom::table_.find(name);
	if (it != table_.end()) {
		if (auto atomPtr = it->second.value().lock()) {
			if (atomPtr->isIRI()) {
				return std::static_pointer_cast<IRIAtom>(atomPtr);
			}
		}
		Atom::table_.erase(it);
	}
	// Atom does not exist or was destroyed, create a new one
	auto inserted = table_.emplace(name, std::nullopt);
	auto &jt = inserted.first;
	auto atom = std::make_shared<knowrob::IRIAtom>(jt->first);
	jt->second = atom;
	auto locked = jt->second.value().lock();
	if (!locked) {
		throw std::runtime_error("Failed to lock IRIAtom");
	}
	return std::static_pointer_cast<IRIAtom>(locked);
}
