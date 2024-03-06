/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string/predicate.hpp>
#include <sstream>
#include "knowrob/queries/SPARQLQuery.h"
#include "knowrob/terms/Atom.h"

using namespace knowrob;

// TODO: use prefixes instead of writing the full IRI. e.g.
//       PREFIX foaf: <http://xmlns.com/foaf/0.1/>
//       SELECT ?name WHERE {
//         _:bnode foaf:name ?name .
//       }

SPARQLQuery::SPARQLQuery(const FramedTriplePattern &triplePattern) : varCounter_(0) {
	std::stringstream os;
	selectBegin(os);
	where(os, triplePattern);
	selectEnd(os);
	queryString_ = os.str();
}

SPARQLQuery::SPARQLQuery(const std::vector<FramedTriplePatternPtr> &triplePatterns) : varCounter_(0) {
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
	static const std::string adhocVarPrefix = "v_adhoc";

	where(os, triplePattern.subjectTerm());
	where(os, triplePattern.propertyTerm());
	// TODO: some context properties are optional. e.g. when query has a confidence threshold, then
	//       all triples without a confidence should be included.
	//       also in case query contains begin and end time, then triples without a begin/end/begin+end time should be included.
	//       maybe also sometimes should include always, and belief knowledge.
	//       BUT how to implement this here? well we could have special code for certain properties here. or try to encode it in the query.
	if(triplePattern.objectOperator() == FramedTriplePattern::OperatorType::EQ) {
		where(os, triplePattern.objectTerm());
		dot(os);
	} else {
		// we need to introduce a temporary variable to handle value expressions like `<(5)` such
		// that they can be filtered after the match.
		auto tempVar = adhocVarPrefix + std::to_string(varCounter_++);
		os << "?" << tempVar << " . ";
		filter(os, tempVar, triplePattern.objectTerm(), triplePattern.objectOperator());
	}
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
			} else if (term->isBlank()) {
				os << "_:" << std::static_pointer_cast<Atomic>(term)->stringForm() << " ";
				break;
			}
		}
		default:
			os << "\"" << *term << "\" ";
			break;
	}
}
