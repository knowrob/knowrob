/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <optional>
#include "knowrob/terms/Blank.h"

using namespace knowrob;

std::shared_ptr<knowrob::Blank> Blank::Tabled(std::string_view name) {
	auto it = Atom::table().find(name);
	if (it != table().end()) {
		if (auto atomPtr = it->second.value().lock()) {
			if (atomPtr->isBlank()) {
				return std::static_pointer_cast<Blank>(atomPtr);
			}
		}
		Atom::table().erase(it);
	}
	// Atom does not exist or was destroyed, create a new one
	auto inserted = table().emplace(name, std::nullopt);
	auto &jt = inserted.first;
	auto atom = std::make_shared<knowrob::Blank>(jt->first);
	jt->second = atom;
	auto locked = jt->second.value().lock();
	if (!locked) {
		throw std::runtime_error("Failed to lock Blank");
	}
	return std::static_pointer_cast<Blank>(locked);
}
