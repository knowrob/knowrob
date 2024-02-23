/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/predicate.hpp>
#include <sstream>
#include "knowrob/queries/SPARQLQuery.h"
#include "knowrob/terms/Atom.h"

using namespace knowrob;

SPARQLQuery::SPARQLQuery(const FramedTriplePattern &triplePattern) {
	std::stringstream os;
	selectBegin(os);
	where(os, triplePattern);
	selectEnd(os);
	queryString_ = os.str();
}

SPARQLQuery::SPARQLQuery(const std::vector<RDFLiteralPtr> &triplePatterns) {
	std::stringstream os;
	selectBegin(os);
	for (const auto &triplePattern: triplePatterns) {
		where(os, *triplePattern);
	}
	selectEnd(os);
	queryString_ = os.str();
}

void SPARQLQuery::selectBegin(std::ostream &os) {
	os << "SELECT * WHERE { ";
}

void SPARQLQuery::selectEnd(std::ostream &os) {
	os << "}";
}

void SPARQLQuery::dot(std::ostream &os) {
	os << ". ";
}

void SPARQLQuery::where(std::ostream &os, const FramedTriplePattern &triplePattern) {
	where(os, triplePattern.subjectTerm());
	where(os, triplePattern.propertyTerm());
	where(os, triplePattern.objectTerm());
	dot(os);
}

void SPARQLQuery::where(std::ostream &os, const TermPtr &term) {
	switch (term->termType()) {
		case TermType::VARIABLE:
			os << "?" << ((Variable *) term.get())->name() << " ";
			break;
		case TermType::ATOMIC: {
			if(term->isIRI()) {
				os << "<" << std::static_pointer_cast<Atomic>(term)->stringForm() << "> ";
				break;
			}
		}
		default:
			os << "\"" << *term << "\" ";
			break;
	}
}
