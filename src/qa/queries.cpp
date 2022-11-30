/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */
 
// SWI Prolog
#include <SWI-Prolog.h>
// KnowRob
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
		phi->readVariables(out);
	}
}


/******************************************/
/********** PrologQueryParser *************/
/******************************************/

static atom_t ATOM_semicolon = PL_new_atom(";");
static atom_t ATOM_comma = PL_new_atom(",");

static std::string PROLOG_LANG_ID = "prolog";

static std::shared_ptr<Predicate> plToPredicate(const term_t &t);
static std::shared_ptr<Term>      plToTerm(const term_t &t);
static std::shared_ptr<Formula>   plToFormula(const term_t &t);
		
const std::string& PrologQueryParser::getLanguageIdentifier() const
{
	return PROLOG_LANG_ID;
}

static std::shared_ptr<Predicate> plToPredicate(const term_t &t)
{
	// get the functor + arity
	size_t arity;
	atom_t name;
	if(!PL_get_name_arity(t, &name, &arity)) {
		throw ParserError("failed to parse input term");
	}
	std::string functorName = PL_atom_chars(name);
	// construct arguments
	std::vector<std::shared_ptr<Term>> arguments(arity);
	term_t arg = PL_new_term_ref();
	for(int n=1; n<=arity; n++) {
		if(PL_get_arg(n, t, arg)) {
			arguments[n-1] = plToTerm(arg);
		}
		else {
			throw ParserError("failed to parse argument of input term");
		}
	}
	// construct Predicate object
	return std::shared_ptr<Predicate>(
		new Predicate(functorName, arguments)
	);
}

static std::shared_ptr<Term> plToTerm(const term_t &t)
{
	Term *tt = NULL;
	
	switch(PL_term_type(t)) {
	case PL_INTEGER: {
		// TODO: check if int64_t/long is needed.
		//          but there seems to be no interface to test this with SWIPL.
		//PL_get_long(t, long *i);
		//PL_get_int64(t, int64_t *i);
		//PL_get_uint64(t, uint64_t *i);
		int val=0;
		if(!PL_get_integer(t, &val)) val=0;
		tt = new Constant<int>(val);
	}
	case PL_FLOAT: {
		double val=0.0;
		if(!PL_get_float(t, &val)) val=0.0;
		tt = new Constant<double>(val);
	}
	case PL_STRING: // TODO: do we need to distinguish betwen atom and string here?
	case PL_ATOM: {
		char *s;
		if(PL_get_chars(t, &s, CVT_ALL)) {
			tt = new Constant<std::string>(std::string(s));
			PL_free(s);
		}
		else {
			tt = new Constant<std::string>("");
		}
	}
	case PL_VARIABLE: {
		// variables are converted to print-name
		char *s;
		if(PL_get_chars(t, &s, CVT_ALL)) {
			tt = new Variable(std::string(s));
			PL_free(s);
		}
		else {
			throw ParserError("failed to parse input variable");
		}
	}
	case PL_TERM:
		return plToPredicate(t);
	default:
		throw ParserError("unknown term type");
	}
	
	return std::shared_ptr<Term>(tt);
}

static std::shared_ptr<Formula> plToFormula(const term_t &t)
{
	// make sure t is a term, other types of terms cannot be a formula
	if(PL_term_type(t) != PL_TERM) {
		throw ParserError("invalid term type (must be a compound)");
	}
	// get the functor + arity
	size_t arity;
	atom_t name;
	if(!PL_get_name_arity(t, &name, &arity)) {
		throw ParserError("failed to parse input term");
	}
	
	// handle conjunctions in queries
	if(arity>1 && name==ATOM_comma) {
		std::vector<std::shared_ptr<Formula>> formulae(arity);
		term_t arg = PL_new_term_ref();
		for(int n=1; n<=arity; n++) {
			if(PL_get_arg(n, t, arg)) {
				formulae[n-1] = plToFormula(arg);
			} else {
				throw ParserError("failed to parse argument of input term");
			}
		}
		// TODO: should be flattened? i.e. if formulae contains a ConjunctionFormula again?
		return std::shared_ptr<Formula>(
			new ConjunctionFormula(formulae));
	}
	// handle disjunctions in queries
	else if(arity>1 && name==ATOM_semicolon) {
		std::vector<std::shared_ptr<Formula>> formulae(arity);
		term_t arg = PL_new_term_ref();
		for(int n=1; n<=arity; n++) {
			if(PL_get_arg(n, t, arg)) {
				formulae[n-1] = plToFormula(arg);
			} else {
				throw ParserError("failed to parse argument of input term");
			}
		}
		// TODO: should be flattened? i.e. if formulae contains a DisjunctionFormula again?
		return std::shared_ptr<Formula>(
			new DisjunctionFormula(formulae));
	}
	// default: a single predicate
	else {
		return std::shared_ptr<Formula>(
			new PredicateFormula(plToPredicate(t)));
	}
}

Query PrologQueryParser::fromString(const std::string &queryString)
{
	term_t t = PL_new_term_ref();
	// construct term_t from query string
	PL_put_atom_chars(t, queryString.c_str());
	// translate term_t into Formula, and construct a query object
	return Query(plToFormula(t));
}


/******************************************/
/************* query results **************/
/******************************************/

void Substitution::set(const Variable &var, const std::shared_ptr<Term> &term)
{
	mapping_[var] = term;
}

std::shared_ptr<Term> Substitution::get(const Variable &var) const
{
	auto it = mapping_.find(var);
	if(it != mapping_.end()) {
		return it->second;
	}
	else {
		return std::shared_ptr<Term>();
	}
}

QueryResult::QueryResult(const std::shared_ptr<Substitution> &substitution)
: hasSolution_(true),
  substitution_(substitution)
{}

QueryResult::QueryResult()
: hasSolution_(false)
{}
		
const QueryResult& QueryResult::noSolution()
{
	static QueryResult instance;
	return instance;
}

