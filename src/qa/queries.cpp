/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/qa/queries.h>

using namespace knowrob;

/******************************************/
/*************** Formulae *****************/
/******************************************/

bool Formula::isAtomic() const
{
	return type() == FormulaType::PREDICATE;
}

std::set<Variable> Formula::getVariables()
{
	std::set<Variable> out;
	readVariables(out);
	return out;
}

void PredicateFormula::readVariables(std::set<Variable> &out)
{
	// TODO: read p_ vars
}

void ConnectiveFormula::readVariables(std::set<Variable> &out)
{
	for(auto &phi : formulae_) {
		phi.readVariables(out);
	}
}

/******************************************/
/********* Prolog query parser ************/
/******************************************/

static atom_t ATOM_semicolon = PL_new_atom(";");
static atom_t ATOM_comma = PL_new_atom(",");
		
const std::string& PrologQueryParser::getLanguageIdentifier() const
{
	static std::string langID = "prolog";
	return langID;
}

static Formula plToFormula(const term_t &t)
{
	// make sure t is a term, other types of terms cannot be a formula
	if(PL_term_type(t) != PL_TERM) {
		throw SomeException;
	}
	// get the functor + arity
	size_t arity;
	atom_t name;
	PL_get_name_arity(t, &name, &arity);
	
	// handle conjunctions in queries
	if(arity>1 && name==ATOM_comma) {
		std::vector<Formula> formulae(arity);
		term_t arg = PL_new_term_ref();
		for(int n=1; n<=arity; n++) {
			PL_get_arg(n, t, arg);
			formulae[n-1] = toFormula(arg);
		}
		// TODO: should be flattened? i.e. if formulae contains a ConjunctionFormula again?
		return ConjunctionFormula(formulae);
	}
	// handle disjunctions in queries
	else if(arity>1 && name==ATOM_semicolon) {
		std::vector<Formula> formulae(arity);
		term_t arg = PL_new_term_ref();
		for(int n=1; n<=arity; n++) {
			PL_get_arg(n, t, arg);
			formulae[n-1] = toFormula(arg);
		}
		// TODO: should be flattened? i.e. if formulae contains a DisjunctionFormula again?
		return DisjunctionFormula(formulae);
	}
	// default: a single predicate
	else {
		return PredicateFormula(toPredicate(t));
	}
}

static Predicate plToPredicate(const term_t &t) const
{
	// get the functor + arity
	size_t arity;
	atom_t name;
	PL_get_name_arity(t, &name, &arity);
	std::string functorName = PL_atom_chars(name);
	// construct arguments
	std::vector<Term> arguments(arity);
	term_t arg = PL_new_term_ref();
	for(int n=1; n<=arity; n++) {
		PL_get_arg(n, t, arg);
		arguments[n-1] = toTerm(arg);
	}
	// construct Predicate object
	return Predicate(functorName, arguments);
}

static Term plToTerm(const term_t &t)
{
	switch(PL_term_type(t)) {
	case PL_INTEGER: {
		// TODO: check if int64_t/long is needed.
		//          but there seems to be no interface to test this with SWIPL.
		//PL_get_long(t, long *i);
		//PL_get_int64(t, int64_t *i);
		//PL_get_uint64(t, uint64_t *i);
		
		int val=0;
		PL_get_integer(t, &val);
		return AtomicTerm<int>(val);
	}
	case PL_FLOAT: {
		double val=0.0;
		PL_get_float(t, &val);
		return AtomicTerm<double>(val);
	}
	case PL_STRING: // TODO: do we need to distinguish betwen atom and string here?
	case PL_ATOM: {
		char *s;
		PL_get_chars(t, &s, CVT_ALL);
		return AtomicTerm<std::string>(std::string(s));
	}
	case PL_VARIABLE: {
		// variables are converted to print-name
		char *s;
		PL_get_chars(t, &s, CVT_ALL);
		return Variable(std::string(s));
	}
	case PL_TERM:
		return toPredicate(t);
	default:
		throw SomeException;
	}
}

boost::shared_ptr<Query> PrologQueryParser::fromString(const std::string &queryString)
{
	term_t t = PL_new_term_ref();
	// construct term_t from query string
	PL_put_atom_chars(t, queryString.c_str());
	// translate term_t into Formula, and construct a query object
	return Query(plToFormula(t));
}
		
// TODO
//std::string toString(const Query &query);

/******************************************/
/************* query results **************/
/******************************************/

QueryResult::QueryResult(const boost::shared_ptr<Substitution> &substitution)
: hasSolution_(true),
  substitution_(substitution)
{}

QueryResult::QueryResult()
: hasSolution_(false)
{}

QueryResult::~QueryResult()
{}
		
static const QueryResult& QueryResult::noSolution()
{
	static QueryResult instance;
	return instance;
}

