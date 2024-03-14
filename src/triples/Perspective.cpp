/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/Perspective.h"
#include "knowrob/py/utils.h"

// TODO: rather let the user configure the ego perspective IRI, or else use fallback
#define EGO_PERSPECTIVE_IRI "http://knowrob.org/kb/knowrob.owl#Self"

using namespace knowrob;

std::map<std::string_view, std::shared_ptr<Perspective>> Perspective::perspectiveMap_ =
	std::map<std::string_view, std::shared_ptr<Perspective>>();

Perspective::Perspective(std::string_view iri)
		: atom_(IRIAtom::Tabled(iri)) {}

std::shared_ptr<Perspective> Perspective::getEgoPerspective() {
	static std::shared_ptr<Perspective> egoPerspective;
	if (egoPerspective == nullptr) {
		egoPerspective = std::make_shared<Perspective>(EGO_PERSPECTIVE_IRI);
		perspectiveMap_[egoPerspective->iri()] = egoPerspective;
	}
	return egoPerspective;
}

bool Perspective::isEgoPerspective(std::string_view iri) {
	return iri.empty() || iri == EGO_PERSPECTIVE_IRI;
}

std::shared_ptr<Perspective> Perspective::get(std::string_view iri) {
	auto it = perspectiveMap_.find(iri);
	if (it == perspectiveMap_.end()) {
		auto agent = std::make_shared<Perspective>(iri);
		perspectiveMap_[iri] = agent;
		return agent;
	} else {
		return it->second;
	}
}

namespace knowrob::py {
	template<>
	void createType<Perspective>() {
		using namespace boost::python;
		class_<Perspective, std::shared_ptr<Perspective>>("Perspective", no_init)
				.def("iri", &Perspective::iri)
				.def("atom", &Perspective::atom)
				.def("getEgoPerspective", &Perspective::getEgoPerspective)
				.def("isEgoPerspective", &Perspective::isEgoPerspective)
				.def("get", &Perspective::get);
	}
}
