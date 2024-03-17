/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/python.hpp>
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/py/utils.h"
#include "knowrob/semweb/Resource.h"
#include "knowrob/semweb/PrefixRegistry.h"

using namespace knowrob;

PredicatePtr IRIAtom::operator()(const TermPtr &s, const TermPtr &o) const {
	auto functor = IRIAtom::Tabled(stringForm());
	return std::make_shared<Predicate>(functor, std::vector<TermPtr>{s, o});
}

std::shared_ptr<knowrob::IRIAtom> IRIAtom::Tabled(std::string_view name) {
	auto it = Atom::table().find(name);
	if (it != table().end()) {
		if (auto atomPtr = it->second.value().lock()) {
			if (atomPtr->isIRI()) {
				return std::static_pointer_cast<IRIAtom>(atomPtr);
			}
		}
		Atom::table().erase(it);
	}
	// Atom does not exist or was destroyed, create a new one
	auto inserted = table().emplace(name, std::nullopt);
	auto &jt = inserted.first;
	auto atom = std::make_shared<knowrob::IRIAtom>(jt->first);
	jt->second = atom;
	auto locked = jt->second.value().lock();
	if (!locked) {
		throw std::runtime_error("Failed to lock IRIAtom");
	}
	return std::static_pointer_cast<IRIAtom>(locked);
}

void IRIAtom::write(std::ostream &os) const {
	auto ns = semweb::Resource::iri_ns(stringForm_);
	if (!ns.empty()) {
		auto alias = PrefixRegistry::uriToAlias(ns);
		if (alias.has_value()) {
			auto name = semweb::Resource::iri_name(stringForm_);
			if (!name.empty()) {
				os << alias.value().get() << ":" << name;
				return;
			}
		}
	}
	Atom::write(os);
}

namespace knowrob::py {
	template<>
	void createType<IRIAtom>() {
		using namespace boost::python;
		class_<IRIAtom, std::shared_ptr<IRIAtom>, bases<Atom, RDFNode>>("IRIAtom", no_init)
				.def("Tabled", &IRIAtom::Tabled)
				.def("rdfNodeType", &IRIAtom::rdfNodeType)
				.def("isIRI", &IRIAtom::isIRI);
	}
}
