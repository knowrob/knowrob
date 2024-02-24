/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/terms/Atom.h"

using namespace knowrob;

Atom::AtomTable& Atom::table() {
	static Atom::AtomTable theTable;
	return theTable;
}

Atom::Atom(std::string_view stringForm)
		: Atomic(), stringForm_(stringForm) {
}

bool Atom::isSameAtom(const Atom &other) const {
	// Atoms can also be constructed without adding them to the table,
	// so we need to compare the string forms.
	return stringForm_ == other.stringForm_;
}

void Atom::write(std::ostream &os) const {
	if (std::islower(stringForm_[0])) {
		// avoid single quotes
		// FIXME: but single quotes are required for atoms with non alpha-numeric characters
		os << stringForm_;
	} else {
		os << '\'' << stringForm_ << '\'';
	}
}

std::shared_ptr<knowrob::Atom> Atom::Tabled(std::string_view name) {
	auto it = table().find(name);
	if (it != table().end()) {
		if (auto atomPtr = it->second.value().lock()) {
			// Atom still exists, return it
			return atomPtr;
		}
		table().erase(it);
	}
	// Atom does not exist or was destroyed, create a new one
	auto inserted = table().emplace(name, std::nullopt);
	auto &jt = inserted.first;
	auto atom = std::make_shared<knowrob::Atom>(jt->first);
	jt->second = atom;
	auto locked = jt->second.value().lock();
	if (!locked) {
		throw std::runtime_error("Failed to lock Atom");
	}
	return locked;
}
