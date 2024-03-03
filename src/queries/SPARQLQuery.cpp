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

SPARQLQuery::SPARQLQuery(const std::vector<FramedTriplePatternPtr> &triplePatterns) {
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

void SPARQLQuery::filter(std::ostream &os, std::string_view varName, const TermPtr &term, FramedTriplePattern::OperatorType operatorType) {
	if (!term->isAtomic()) return;
	auto atomic = std::static_pointer_cast<Atomic>(term);
	os << "FILTER (?" << varName;
	switch (operatorType) {
		case FramedTriplePattern::OperatorType::LT:
			os << " < ";
			break;
		case FramedTriplePattern::OperatorType::GT:
			os << " > ";
			break;
		case FramedTriplePattern::OperatorType::LEQ:
			os << " <= ";
			break;
		case FramedTriplePattern::OperatorType::GEQ:
			os << " >= ";
			break;
		case FramedTriplePattern::EQ:
			os << " = ";
			break;
	}
	os << atomic->stringForm() << ") ";
}

void SPARQLQuery::where(std::ostream &os, const FramedTriplePattern &triplePattern) {
	where(os, triplePattern.subjectTerm());
	where(os, triplePattern.propertyTerm());
	// TODO: some context properties are optional. e.g. when query has a confidence threshold, then
	//       all triples without a confidence should be included.
	//       also in case query contains begin and end time, then triples without a begin/end/begin+end time should be included.
	//       maybe also sometimes should include always, and belief knowledge.
	//       BUT how to implement this here? well we could have special code for certain properties here. or try to encode it in the query.
	where(os, triplePattern.objectTerm());
	// TODO: need to introduce a temporary variable to handle value expressions like `<(5)`.
	//            can also support terms like `<(5)->Var` where the value is fixed and the variable is the rest.
	//filter(os, triplePattern.objectTerm(), tempVar, triplePattern.objectOperator());
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
			// TODO: encoding of blanks in SPARQL
		}
		default:
			os << "\"" << *term << "\" ";
			break;
	}
}
