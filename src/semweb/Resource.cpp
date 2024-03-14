/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/Resource.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/Logger.h"
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iomanip>

using namespace knowrob::semweb;

Resource::Resource(std::string_view iri) {
	switch (rdfNodeTypeGuess(iri)) {
		case RDFNodeType::BLANK:
			iri_ = Blank::Tabled(iri);
			break;
		case RDFNodeType::IRI:
			iri_ = IRIAtom::Tabled(iri);
			break;
		case RDFNodeType::LITERAL:
			KB_WARN("Resource created with guessed literal type: {}. Treating as IRI.", iri);
			iri_ = IRIAtom::Tabled(iri);
			break;
	}
}

static inline void write_unique(std::ostream &os) {
	static boost::uuids::random_generator generator;
	std::hash<std::string> str_hash;
	os << ' ';
	os << std::setfill('0') << std::setw(8) << std::hex <<
	   str_hash(to_string(generator()));
}

knowrob::IRIAtomPtr Resource::unique_iri(std::string_view ns, std::string_view name) {
	std::stringstream ss;
	ss << ns;
	if (!ns.empty() && ns.back() != '#') {
		ss << "#";
	}
	write_unique(ss);
	return IRIAtom::Tabled(ss.str());
}

knowrob::IRIAtomPtr Resource::unique_iri(std::string_view type_iri) {
	std::stringstream ss;
	ss << type_iri;
	write_unique(ss);
	return IRIAtom::Tabled(ss.str());
}

std::string_view Resource::iri_name(std::string_view iri) {
	auto pos = iri.find('#');
	if (pos != std::string::npos) {
		return iri.substr(pos + 1);
	}
	return {iri.data()};
}

std::string_view Resource::name() const {
	return iri_name(iri_->stringForm());
}

std::string_view Resource::iri_ns(std::string_view iri, bool includeDelimiter) {
	auto pos = iri.rfind('#');
	if (pos != std::string::npos) {
		auto pos_x = (includeDelimiter ? pos + 1 : pos);
		return iri.substr(0, pos_x);
	}
	return {};
}

std::string_view Resource::ns(bool includeDelimiter) const {
	return iri_ns(iri_->stringForm(), includeDelimiter);
}
