/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <spdlog/spdlog.h>

#include <knowrob/reasoning/prolog/PrologQuery.h>

static atom_t ATOM_semicolon = PL_new_atom(";");
static atom_t ATOM_comma     = PL_new_atom(",");
static atom_t ATOM_fail      = PL_new_atom("fail");
static atom_t ATOM_false     = PL_new_atom("false");
static atom_t ATOM_true      = PL_new_atom("true");

using namespace knowrob;

PrologQuery::PrologQuery(const std::shared_ptr<Query> &qa_query)
: qa_query_(qa_query),
  pl_query_(PL_new_term_ref())
{
	constructPrologTerm(qa_query->formula(), pl_query_);
	createPrologPredicate();
}

PrologQuery::PrologQuery(const std::string &queryString)
: pl_query_(PL_new_term_ref())
{
	// construct term_t from query string
	if(!PL_put_atom_chars(pl_query_, queryString.c_str())) {
		spdlog::error("PL_put_atom_chars failed.");
		throw ParserError("Parsing of t_term failed.");
	}
	// translate term_t into Formula, and construct a query object
	qa_query_ = std::shared_ptr<Query>(
		new Query(PrologQuery::constructFormula(pl_query_)));
	createPrologPredicate();
}

PrologQuery::PrologQuery(const term_t &pl_query)
: pl_query_(PL_new_term_ref())
{
	if(!PL_put_term(pl_query_, pl_query)) {
		spdlog::error("PL_put_term failed.");
		throw ParserError("Parsing of t_term failed.");
	}
	qa_query_ = std::shared_ptr<Query>(
		new Query(PrologQuery::constructFormula(pl_query_)));
	createPrologPredicate();
}

PrologQuery::~PrologQuery()
{
	PL_reset_term_refs(pl_query_);
}

void PrologQuery::createPrologPredicate()
{
	static predicate_t comma_predicate     = PL_pred(PL_new_functor(ATOM_comma, 2), NULL);
	static predicate_t semicolon_predicate = PL_pred(PL_new_functor(ATOM_semicolon, 2), NULL);
	
	// get the predicate indicator.
	// pl_query_ is a term, but open query only accepts predicates.
	// so the goal here is to create a predicate using the functor of pl_query_.
	switch(qa_query_->formula()->type()) {
	case FormulaType::PREDICATE: {
		const std::shared_ptr<Predicate> &qa_pred =
			((PredicateFormula*) qa_query_->formula().get())->predicate();
		// TODO: should the predicate_t be stored in a static variable?
		pl_predicate_ = PL_predicate(
			qa_pred->indicator().functor().c_str(),
			qa_pred->arguments().size(),
			NULL);
		break;
	}
	case FormulaType::CONJUNCTION:
		pl_predicate_ = comma_predicate;
		break;
	case FormulaType::DISJUNCTION:
		pl_predicate_ = semicolon_predicate;
		break;
	default:
		spdlog::warn("Ignoring unknown formula type '{}'.", qa_query_->formula()->type());
		break;
	}
	// make pl_arguments_ point to first argument of pl_query_
	if(!PL_get_arg(0, pl_query_, pl_arguments_)) {
		spdlog::warn("failed to get query argument term");
	}
}

void PrologQuery::constructPrologTerm(
	const std::shared_ptr<Formula>& phi,
	term_t &pl_term)
{
	static functor_t comma_functor = PL_new_functor(ATOM_comma, 2);
	static functor_t semicolon_functor = PL_new_functor(ATOM_semicolon, 2);
	
	switch(phi->type()) {
	case FormulaType::PREDICATE: {
		const std::shared_ptr<Predicate> &qa_pred =
			((PredicateFormula*) phi.get())->predicate();
		constructPrologTerm(qa_pred, pl_term);
		break;
	}
	case FormulaType::CONJUNCTION:
		constructPrologTerm(
			(ConnectiveFormula*)phi.get(),
			comma_functor, pl_term);
		break;
	case FormulaType::DISJUNCTION:
		constructPrologTerm(
			(ConnectiveFormula*)phi.get(),
			semicolon_functor, pl_term);
		break;
	default:
		spdlog::warn("Ignoring unknown formula type '{}' in query.", phi->type());
		break;
	}
}

void PrologQuery::constructPrologTerm(
	const std::shared_ptr<Term>& qa_term,
	term_t &pl_term)
{
	switch(qa_term->type()) {
	case TermType::VARIABLE: {
		Variable *qa_var = (Variable*)qa_term.get();
		// try to use previously created term_t
		auto it = vars_.find(qa_var->name());
		if(it != vars_.end()) {
			if(!PL_put_term(pl_term, it->second)) {
				spdlog::warn("PL_put_term failed for variable {}.", qa_var->name());
			}
		}
		// create a new variable
		else if(PL_put_variable(pl_term)) {
			vars_[qa_var->name()] = pl_term;
		}
		break;
	}
	case TermType::PREDICATE: {
		Predicate *qa_pred = (Predicate*)qa_term.get();
		if(qa_pred->indicator().arity()>0) {
			// create a term reference for each argument of qa_pred
			term_t pl_arg = PL_new_term_refs(qa_pred->indicator().arity());
			term_t pl_arg0 = pl_arg;
			// construct argument terms
			for(const std::shared_ptr<Term> &qa_arg : qa_pred->arguments()) {
				constructPrologTerm(qa_arg, pl_arg);
				pl_arg += 1;
			}
			// construct output term
			if(!PL_cons_functor_v(pl_term,
				PL_new_functor(
					PL_new_atom(qa_pred->indicator().functor().c_str()),
					qa_pred->indicator().arity()),
				pl_arg0)) {
				spdlog::warn("PL_cons_functor_v failed for functor {}.", qa_pred->indicator().functor());
			}
		}
		else {
			// 0-ary predicates are atoms
			if(!PL_put_atom_chars(pl_term, qa_pred->indicator().functor().c_str())) {
				spdlog::warn("PL_put_atom_chars failed for string {}.", qa_pred->indicator().functor());
			}
		}
		break;
	}
	case TermType::STRING:
		if(!PL_put_atom_chars(pl_term, ((StringTerm*)qa_term.get())->value().c_str())) {
			spdlog::warn("PL_put_atom_chars failed for string {}.", ((StringTerm*)qa_term.get())->value());
		}
		break;
	case TermType::DOUBLE:
		if(!PL_put_float(pl_term, ((DoubleTerm*)qa_term.get())->value())) {
			spdlog::warn("PL_put_float failed for value {}.", ((DoubleTerm*)qa_term.get())->value());
		}
		break;
	case TermType::INT32:
		if(!PL_put_integer(pl_term, ((Integer32Term*)qa_term.get())->value())) {
			spdlog::warn("PL_put_integer failed for value {}.", ((Integer32Term*)qa_term.get())->value());
		}
		break;
	case TermType::LONG:
		if(!PL_put_integer(pl_term, ((LongTerm*)qa_term.get())->value())) {
			spdlog::warn("PL_put_integer failed for value {}.", ((LongTerm*)qa_term.get())->value());
		}
		break;
	case TermType::BOTTOM:
		if(!PL_put_atom_chars(pl_term, PL_atom_chars(ATOM_fail))) {
			spdlog::warn("PL_put_atom_chars failed for 'fail' atom.");
		}
		break;
	case TermType::TOP:
		if(!PL_put_atom_chars(pl_term, PL_atom_chars(ATOM_true))) {
			spdlog::warn("PL_put_atom_chars failed for 'true' atom.");
		}
		break;
	default:
		spdlog::warn("Ignoring unknown term type '{}' in query.", qa_term->type());
		break;
	}
}

void PrologQuery::constructPrologTerm(
	ConnectiveFormula *phi,
	functor_t &pl_functor,
	term_t &pl_term)
{
	term_t pl_pred = pl_term;
	int counter = 1;
	
	for(const auto &psi : phi->formulae()) {
		if(counter < phi->formulae().size()) {
			// create a fresh term for each argument of the operator
			term_t pl_arg = PL_new_term_refs(2);
			// construct the first argument
			constructPrologTerm(psi, pl_arg);
			// construct the operator predicate (leave second argument unspecified)
			if(!PL_cons_functor_v(pl_pred, pl_functor, pl_arg)) {
				spdlog::warn("Failed to construct Prolog term for a formula.");
			}
			// second argument is the list constructed in the next step 
			pl_pred = (pl_arg+1);
		}
		else {
			// last formula
			constructPrologTerm(psi, pl_pred);
		}
		counter += 1;
	}
}

std::shared_ptr<Term> PrologQuery::constructPredicate(const term_t &t)
{
	// get the functor + arity
	size_t arity;
	atom_t name;
	if(!PL_get_name_arity(t, &name, &arity)) {
		spdlog::error("Failed to read term_t name and arity.");
		return BottomTerm::get();
	}
	std::string functorName = PL_atom_chars(name);
	// construct arguments
	std::vector<std::shared_ptr<Term>> arguments(arity);
	term_t arg = PL_new_term_ref();
	for(int n=1; n<=arity; n++) {
		if(PL_get_arg(n, t, arg)) {
			arguments[n-1] = constructTerm(arg);
		}
		else {
			spdlog::error("Failed to construct argument {} of predicate {}.", n, functorName);
			return BottomTerm::get();
		}
	}
	// construct Predicate object
	return std::shared_ptr<Predicate>(
		new Predicate(functorName, arguments));
}

std::shared_ptr<Term> PrologQuery::constructTerm(const term_t &t)
{
	Term *tt = NULL;
	
	switch(PL_term_type(t)) {
	case PL_INTEGER: {
		long val=0;
		if(!PL_get_long(t, &val)) {
			spdlog::error("Failed to read long from term {}.", t);
			return BottomTerm::get();
		}
		tt = new LongTerm(val);
	}
	case PL_FLOAT: {
		double val=0.0;
		if(!PL_get_float(t, &val)) {
			spdlog::error("Failed to read double from term {}.", t);
			return BottomTerm::get();
		}
		tt = new DoubleTerm(val);
	}
	case PL_STRING: {
		char *s;
		if(!PL_get_chars(t, &s, CVT_ALL)) {
			spdlog::error("Failed to read string from term {}.", t);
			return BottomTerm::get();
		}
		tt = new StringTerm(std::string(s));
		PL_free(s);
	}
	case PL_ATOM: {
		atom_t atom;
		if(!PL_get_atom(t, &atom)) {
			spdlog::error("failed to read atom from term {}.", t);
			return BottomTerm::get();
		}
		
		// map `fail/0` and `false/0` to BottomTerm
		if(atom == ATOM_fail || atom == ATOM_false) {
			return BottomTerm::get();
		}
		// map `true/0` to TopTerm
		else if(atom == ATOM_true) {
			return TopTerm::get();
		}
		
		tt = new StringTerm(std::string(PL_atom_chars(atom)));
	}
	case PL_VARIABLE: {
		// variables are converted to print-name
		char *s;
		if(!PL_get_chars(t, &s, CVT_ALL)) {
			spdlog::error("Failed to read variable name from term {}.", t);
			return BottomTerm::get();
		}
		tt = new Variable(std::string(s));
		PL_free(s);
	}
	case PL_TERM:
		return PrologQuery::constructPredicate(t);
	default:
		spdlog::error("unknown term type {}.", PL_term_type(t));
		return BottomTerm::get();
	}
	
	return std::shared_ptr<Term>(tt);
}

std::shared_ptr<Formula> PrologQuery::constructFormula(const term_t &t)
{
	// make sure t is a term, other types of terms cannot be a formula
	if(PL_term_type(t) != PL_TERM) {
		spdlog::error("Invalid term type {} (must be a compound).", PL_term_type(t));
		throw ParserError("Parsing of t_term failed.");
	}
	// get the functor + arity
	size_t arity;
	atom_t name;
	if(!PL_get_name_arity(t, &name, &arity)) {
		throw ParserError("Parsing of t_term failed.");
	}
	
	// handle conjunctions in queries
	if(arity>1 && name==ATOM_comma) {
		std::vector<std::shared_ptr<Formula>> formulae(arity);
		term_t arg = PL_new_term_ref();
		for(int n=1; n<=arity; n++) {
			if(PL_get_arg(n, t, arg)) {
				formulae[n-1] = PrologQuery::constructFormula(arg);
			} else {
				spdlog::error("Failed to parse argument {} of input term {}.", n, t);
				throw ParserError("Parsing of t_term failed.");
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
				formulae[n-1] = PrologQuery::constructFormula(arg);
			} else {
				spdlog::error("Failed to parse argument {} of input term {}.", n, t);
				throw ParserError("Parsing of t_term failed.");
			}
		}
		// TODO: should be flattened? i.e. if formulae contains a DisjunctionFormula again?
		return std::shared_ptr<Formula>(
			new DisjunctionFormula(formulae));
	}
	// default: a single predicate
	else {
		std::shared_ptr<Term> p = PrologQuery::constructPredicate(t);
		if(p->isBottom()) {
			throw ParserError("Parsing of t_term failed.");
		}
		else {
			return std::shared_ptr<Formula>(new PredicateFormula(
				std::static_pointer_cast<Predicate>(p)));
		}
	}
}

