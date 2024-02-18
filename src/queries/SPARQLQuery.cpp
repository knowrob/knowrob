/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/predicate.hpp>
#include <sstream>
#include "knowrob/queries/SPARQLQuery.h"

using namespace knowrob;

SPARQLQuery::SPARQLQuery(const RDFLiteral &triplePattern) {
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

void SPARQLQuery::where(std::ostream &os, const RDFLiteral &triplePattern) {
	where(os, triplePattern.subjectTerm());
	where(os, triplePattern.propertyTerm());
	where(os, triplePattern.objectTerm());
	dot(os);
}

void SPARQLQuery::where(std::ostream &os, const TermPtr &term) {
	switch (term->type()) {
		case TermType::VARIABLE:
			os << "?" << ((Variable *) term.get())->name() << " ";
			break;
		case TermType::STRING: {
			// FIXME: need to distinguish strings and resources! below is a temporary HACK
			auto str = term->toString();
			if (boost::algorithm::starts_with(str, "http") ||
				boost::algorithm::starts_with(str, "_")) {
				os << "<" << *term << "> ";
				break;
			}
		}
		default:
			os << "\"" << *term << "\" ";
			break;
	}
}
