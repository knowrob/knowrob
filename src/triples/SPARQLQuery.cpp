/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <sstream>
#include "knowrob/triples/SPARQLQuery.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/triples/GraphSequence.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/semweb/Resource.h"

using namespace knowrob;

// TODO: build a SELECT pattern rather then using "*" to return all variables.

SPARQLQuery::SPARQLQuery(const FramedTriplePattern &triplePattern) : varCounter_(0) {
	std::stringstream os, os_query;
	selectBegin(os_query);
	add(os_query, triplePattern);
	selectEnd(os_query);

	appendPrefixes(os);
	os << os_query.str();
	queryString_ = os.str();
}

SPARQLQuery::SPARQLQuery(const std::shared_ptr<GraphQuery> &query) : varCounter_(0) {
	std::stringstream os_query;
	selectBegin(os_query);
	add(os_query, query->term());
	selectEnd(os_query);

	std::stringstream os;
	appendPrefixes(os);
	os << os_query.str();

	if (query->ctx()->queryFlags & QUERY_FLAG_ONE_SOLUTION) {
		os << "\nLIMIT 1";
	}

	queryString_ = os.str();
}

void SPARQLQuery::appendPrefixes(std::ostream &os) {
	for (const auto &[alias, uri]: aliases_) {
		os << "PREFIX " << alias << ": <" << uri << "#>\n";
	}
}


void SPARQLQuery::add(std::ostream &os, const std::shared_ptr<GraphTerm> &graphTerm) { // NOLINT
	switch (graphTerm->termType()) {
		case GraphTermType::Union: {
			auto sequence = std::static_pointer_cast<GraphSequence>(graphTerm);
			bool isFirst = true;
			for (const auto &term: sequence->terms()) {
				if (!isFirst) {
					os << "  UNION\n";
				} else {
					isFirst = false;
				}
				os << "  {\n";
				add(os, term);
				os << "  }\n";
			}
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
	os << "    ";
	if (triplePattern.isNegated()) {
		// Handle negated patterns. There is literature available for negation in SPARQL:
		// 		https://ceur-ws.org/Vol-1644/paper11.pdf
		// The authors list some approaches:
		// - use OPTIONAL and then check with BOUND that it failed. But that would only do the trick
		//   if the negated pattern actually has some runtime variables, as far as I understand.
		// - use FILTER NOT EXISTS: `FILTER NOT EXISTS { ?x ?y ?z }`.
		// - use MINUS: `MINUS { ?x ?y ?z }`.
		// NOTE: this will not work without a positive statement preceding the negated one
		// as far as I understand because both can only be used to "eliminate" solutions that were produced before.
		// Which is actually fine as the KB does order positive/negative literals in the query,
		// However queries with only negated patterns will not work with this code!
		// NOTE: redland does not support NOT-EXISTS or MINUS.
		// TODO: support different ways of stating negation depending on engine capabilities.
		//negationViaMinus(os, triplePattern);
		negationViaOptional(os, triplePattern);
	} else if (triplePattern.isOptional()) {
		if (optional(os, triplePattern)) {
			filter_optional(os, lastVar_, triplePattern.objectTerm(), triplePattern.objectOperator());
		}
	} else {
		where_with_filter(os, triplePattern);
	}
	os << '\n';
}

void SPARQLQuery::comparison(std::ostream &os, const GraphBuiltin &builtin, const char *comparisonOperator) {
	// Filter using a comparison operator, also succeed if one of the arguments is not bound.
	// e.g. `FILTER (!BOUND(?begin) || !BOUND(?end) || ?begin < ?end)`
	os << "FILTER ( ";
	if (builtin.arguments()[0]->isVariable()) {
		os << "!BOUND(?" << std::static_pointer_cast<Variable>(builtin.arguments()[0])->name() << ") || ";
	}
	if (builtin.arguments()[1]->isVariable()) {
		os << "!BOUND(?" << std::static_pointer_cast<Variable>(builtin.arguments()[1])->name() << ") || ";
	}
	where(os, builtin.arguments()[0]);
	os << comparisonOperator << ' ';
	where(os, builtin.arguments()[1]);
	os << ") ";
}

void SPARQLQuery::bindOneOfIf(std::ostream &os, const GraphBuiltin &builtin, const char *comparisonOperator) {
	// e.g. `BIND(IF(BOUND(?optional_val) && !(?begin < ?optional_val), ?optional_val, ?begin) AS ?begin)`
	// NOTE: Here it is assumed that only the second argument could be undefined in the evaluation context.
	os << "BIND (IF(";
	if (builtin.arguments()[1]->isVariable()) {
		os << "BOUND(?" << std::static_pointer_cast<Variable>(builtin.arguments()[1])->name() << ") && ";
	}
	os << "!(";
	{
		where(os, builtin.arguments()[0]);
		os << ' ' << comparisonOperator << ' ';
		where(os, builtin.arguments()[1]);
	}
	os << "), ";
	where(os, builtin.arguments()[1]);
	os << ", ";
	where(os, builtin.arguments()[0]);
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
	os << "SELECT *\nWHERE {\n";
}

void SPARQLQuery::selectEnd(std::ostream &os) {
	os << "}";
}

void SPARQLQuery::filter_optional(std::ostream &os, std::string_view varName, const TermPtr &term,
								  FramedTriplePattern::OperatorType operatorType) {
	if (!term->isAtomic()) return;
	os << "FILTER (";
	os << " !BOUND(?" << varName << ") || ";
	doFilter(os, varName, std::static_pointer_cast<Atomic>(term), operatorType);
	os << ") ";
}

void SPARQLQuery::filter(std::ostream &os, std::string_view varName, const TermPtr &term,
						 FramedTriplePattern::OperatorType operatorType) {
	if (!term->isAtomic()) return;
	auto atomic = std::static_pointer_cast<Atomic>(term);
	os << "FILTER (";
	doFilter(os, varName, std::static_pointer_cast<Atomic>(term), operatorType);
	os << ") ";
}

void SPARQLQuery::doFilter(std::ostream &os, std::string_view varName, const std::shared_ptr<Atomic> &atomic,
						   FramedTriplePattern::OperatorType operatorType) {
	os << '?' << varName;
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
	os << atomic->stringForm() << " ";
}

void SPARQLQuery::negationViaNotExists(std::ostream &os, const FramedTriplePattern &triplePattern) {
	os << "FILTER NOT EXISTS { ";
	where_with_filter(os, triplePattern);
	os << "} ";
}

void SPARQLQuery::negationViaMinus(std::ostream &os, const FramedTriplePattern &triplePattern) {
	os << "MINUS { ";
	where_with_filter(os, triplePattern);
	os << "} ";
}

void SPARQLQuery::negationViaOptional(std::ostream &os, const FramedTriplePattern &triplePattern) {
	if (triplePattern.objectTerm()->isVariable()) {
		bool hasObjectOperator = optional(os, triplePattern);
		// TODO: Handle object operators here. Best would be to use filter with inverse operator instead of BOUND below.
		os << "FILTER ( !BOUND(";
		where(os, triplePattern.objectTerm());
		os << ")) ";
	} else {
		KB_WARN("Negation via optional is only supported for variable objects.");
	}
}

bool SPARQLQuery::optional(std::ostream &os, const FramedTriplePattern &triplePattern) {
	os << "OPTIONAL { ";
	bool needsFilter = where(os, triplePattern);
	os << "} ";
	return needsFilter;
}

void SPARQLQuery::iri(std::ostream &os, std::string_view iri) {
	auto ns = semweb::Resource::iri_ns(iri);
	auto alias = semweb::PrefixRegistry::uriToAlias(ns);
	if (alias.has_value()) {
		auto &v_alias = alias.value().get();
		os << v_alias << ':' << semweb::Resource::iri_name(iri);
		aliases_[v_alias] = ns;
	} else {
		os << '<' << iri << '>';
	}
	os << ' ';
}

void SPARQLQuery::where_with_filter(std::ostream &os, const FramedTriplePattern &triplePattern) {
	if (where(os, triplePattern)) {
		filter(os, lastVar_, triplePattern.objectTerm(), triplePattern.objectOperator());
	}
}

bool SPARQLQuery::where(std::ostream &os, const FramedTriplePattern &triplePattern) {
	static const std::string adhocVarPrefix = "v_adhoc";

	where(os, triplePattern.subjectTerm());
	where(os, triplePattern.propertyTerm());
	if (triplePattern.objectTerm()->isVariable() ||
		triplePattern.objectOperator() == FramedTriplePattern::OperatorType::EQ) {
		where(os, triplePattern.objectTerm());
		os << ". ";
		return false;
	} else {
		// we need to introduce a temporary variable to handle value expressions like `<(5)` such
		// that they can be filtered after the match.
		lastVar_ = adhocVarPrefix + std::to_string(varCounter_++);
		os << "?" << lastVar_ << " . ";
		return true;
	}
}

void SPARQLQuery::where(std::ostream &os, const TermPtr &term) {
	switch (term->termType()) {
		case TermType::VARIABLE:
			os << "?" << ((Variable *) term.get())->name() << " ";
			break;
		case TermType::ATOMIC: {
			if (term->isIRI()) {
				iri(os, std::static_pointer_cast<Atomic>(term)->stringForm());
				break;
			} else if (term->isBlank()) {
				os << "_:" << std::static_pointer_cast<Atomic>(term)->stringForm() << " ";
				break;
			}
		}
		default:
			if (term->isNumeric() || term->isString()) {
				auto xsdAtomic = std::static_pointer_cast<XSDAtomic>(term);
				if (xsdAtomic->xsdType() == XSDType::BOOLEAN) {
					auto numeric = std::static_pointer_cast<Numeric>(term);
					os << (numeric->asBoolean() ? "\"true\"" : "\"false\"");
				} else {
					os << *term;
				}
				os << "^^";
				iri(os, xsdTypeToIRI(xsdAtomic->xsdType()));
				os << " ";
			}
			break;
	}
}
