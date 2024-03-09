/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <sstream>
#include "knowrob/triples/SPARQLQuery.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/triples/GraphSequence.h"

using namespace knowrob;

// TODO: use prefixes instead of writing the full IRI. e.g.
//       PREFIX foaf: <http://xmlns.com/foaf/0.1/>
//       SELECT ?name WHERE {
//         _:bnode foaf:name ?name .
//       }

SPARQLQuery::SPARQLQuery(const FramedTriplePattern &triplePattern) : varCounter_(0) {
	std::stringstream os;
	selectBegin(os);
	add(os, triplePattern);
	selectEnd(os);
	queryString_ = os.str();
}

SPARQLQuery::SPARQLQuery(const std::shared_ptr<GraphQuery> &query) : varCounter_(0) {
	std::stringstream os;
	selectBegin(os);
	add(os, query->term());
	selectEnd(os);
	queryString_ = os.str();
}

void SPARQLQuery::add(std::ostream &os, const std::shared_ptr<GraphTerm> &graphTerm) { // NOLINT
	switch (graphTerm->termType()) {
		case GraphTermType::Union: {
			auto sequence = std::static_pointer_cast<GraphSequence>(graphTerm);
			bool isFirst = true;
			os << "{ ";
			for (const auto &term: sequence->terms()) {
				if (!isFirst) {
					os << " UNION ";
				} else {
					isFirst = false;
				}
				os << "{ ";
				add(os, term);
				os << "} ";
			}
			os << "} ";
			break;

		}
		case GraphTermType::Sequence: {
			auto sequence = std::static_pointer_cast<GraphSequence>(graphTerm);
			for (const auto &term: sequence->terms()) {
				add(os, term);
			}
			break;
		}
		case GraphTermType::Pattern:
			add(os, *std::static_pointer_cast<GraphPattern>(graphTerm)->value());
			break;
		case GraphTermType::Builtin:
			add(os, *std::static_pointer_cast<GraphBuiltin>(graphTerm));
			break;
	}
}

void SPARQLQuery::add(std::ostream &os, const FramedTriplePattern &triplePattern) {
	if (triplePattern.isNegated()) {
		filterNotExists(os, triplePattern);
	} else if (triplePattern.isOptional()) {
		optional(os, triplePattern);
	} else {
		where(os, triplePattern);
	}
}

void SPARQLQuery::comparison(std::ostream &os, const GraphBuiltin &builtin, const char *comparisonOperator) {
	// e.g. `FILTER (?begin < ?end)`
	os << "FILTER (";
	where(os, builtin.arguments()[0]);
	os << comparisonOperator << ' ';
	where(os, builtin.arguments()[1]);
	os << ") ";
}

void SPARQLQuery::bindOneOfIf(std::ostream &os, const GraphBuiltin &builtin, const char *comparisonOperator) {
	// e.g. `BIND(IF(?begin > ?next_begin, ?begin, ?next_begin) AS ?begin)`
	os << "BIND (IF(";
	where(os, builtin.arguments()[0]);
	os << ' ' << comparisonOperator << ' ';
	where(os, builtin.arguments()[1]);
	os << ", ";
	where(os, builtin.arguments()[0]);
	os << ", ";
	where(os, builtin.arguments()[1]);
	os << ") AS ?" << builtin.bindVar()->name() << ") ";
}

void SPARQLQuery::add(std::ostream &os, const GraphBuiltin &builtin) {
	switch (builtin.builtinType()) {
		case GraphBuiltinType::Equal:
			comparison(os, builtin, "=");
			break;
		case GraphBuiltinType::Less:
			comparison(os, builtin, "<");
			break;
		case GraphBuiltinType::Greater:
			comparison(os, builtin, ">");
			break;
		case GraphBuiltinType::LessOrEqual:
			comparison(os, builtin, "<=");
			break;
		case GraphBuiltinType::GreaterOrEqual:
			comparison(os, builtin, ">=");
			break;
		case GraphBuiltinType::Bind:
			os << "BIND (";
			where(os, builtin.arguments()[0]);
			os << " AS ?" << builtin.bindVar()->name();
			os << ") ";
			break;
		case GraphBuiltinType::Min:
			bindOneOfIf(os, builtin, "<");
			break;
		case GraphBuiltinType::Max:
			bindOneOfIf(os, builtin, ">");
			break;
	}
}

void SPARQLQuery::selectBegin(std::ostream &os) {
	os << "SELECT * WHERE { ";
}

void SPARQLQuery::selectEnd(std::ostream &os) {
	os << "}";
}

void SPARQLQuery::filter(std::ostream &os, std::string_view varName, const TermPtr &term,
						 FramedTriplePattern::OperatorType operatorType) {
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

void SPARQLQuery::filterNotExists(std::ostream &os, const FramedTriplePattern &triplePattern) {
	os << "FILTER NOT EXISTS { ";
	where(os, triplePattern);
	os << "} ";
}

void SPARQLQuery::optional(std::ostream &os, const FramedTriplePattern &triplePattern) {
	os << "OPTIONAL { ";
	where(os, triplePattern);
	os << "} ";
}

void SPARQLQuery::where(std::ostream &os, const FramedTriplePattern &triplePattern) {
	static const std::string adhocVarPrefix = "v_adhoc";

	where(os, triplePattern.subjectTerm());
	where(os, triplePattern.propertyTerm());
	if (triplePattern.objectOperator() == FramedTriplePattern::OperatorType::EQ) {
		where(os, triplePattern.objectTerm());
		os << ". ";
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
			if (term->isIRI()) {
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
