/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <spdlog/spdlog.h>

#include <knowrob/reasoning/prolog/PrologQuery.h>

using namespace knowrob;

PrologQuery::PrologQuery(const std::shared_ptr<Query> &qa_query)
: qa_query_(qa_query),
  pl_query_(PL_new_term_ref())
{
	if(constructPrologTerm(qa_query->formula(), pl_query_)) {
		createPrologPredicate();
	}
	else {
		throw ParserError("constructPrologTerm failed.");
	}
}

PrologQuery::~PrologQuery()
{
	PL_reset_term_refs(pl_arguments_);
	PL_reset_term_refs(pl_query_);
}

void PrologQuery::createPrologPredicate()
{
	static atom_t ATOM_semicolon = PL_new_atom(";");
	static atom_t ATOM_comma     = PL_new_atom(",");
	static predicate_t comma_predicate     = PL_pred(PL_new_functor(ATOM_comma, 2), NULL);
	static predicate_t semicolon_predicate = PL_pred(PL_new_functor(ATOM_semicolon, 2), NULL);
	
	uint32_t arity=0;
	
	// get the predicate indicator.
	// pl_query_ is a term, but open query only accepts predicates.
	// so the goal here is to create a predicate using the functor of pl_query_.
	switch(qa_query_->formula()->type()) {
	case FormulaType::PREDICATE: {
		const std::shared_ptr<Predicate> &qa_pred =
			((PredicateFormula*) qa_query_->formula().get())->predicate();
		arity = qa_pred->arguments().size();
		// TODO: should the predicate_t be stored in a static variable?
		pl_predicate_ = PL_predicate(
			qa_pred->indicator().functor().c_str(), arity, NULL);
		break;
	}
	case FormulaType::CONJUNCTION:
		pl_predicate_ = comma_predicate;
		arity = 2;
		break;
	case FormulaType::DISJUNCTION:
		pl_predicate_ = semicolon_predicate;
		arity = 2;
		break;
	default:
		spdlog::warn("Ignoring unknown formula type '{}'.", (int)qa_query_->formula()->type());
		break;
	}
	
	// create pl_arguments_
  	pl_arguments_ = PL_new_term_refs(arity);
  	for(uint32_t i=0; i<arity; ++i) {
  		term_t arg = pl_arguments_ + i;
		if(!PL_get_arg(i+1, pl_query_, arg)) {
			spdlog::warn("failed to get query argument term");
		}
  	}
}

bool PrologQuery::constructPrologTerm(const std::shared_ptr<Formula>& phi, term_t &pl_term)
{
	static atom_t ATOM_semicolon = PL_new_atom(";");
	static atom_t ATOM_comma     = PL_new_atom(",");
	static functor_t comma_functor     = PL_new_functor(ATOM_comma, 2);
	static functor_t semicolon_functor = PL_new_functor(ATOM_semicolon, 2);
	
	switch(phi->type()) {
	case FormulaType::PREDICATE: {
		const std::shared_ptr<Predicate> &qa_pred =
			((PredicateFormula*) phi.get())->predicate();
		return constructPrologTerm(qa_pred, pl_term);
	}
	case FormulaType::CONJUNCTION:
		return constructPrologTerm(
			(ConnectiveFormula*)phi.get(), comma_functor, pl_term);
	case FormulaType::DISJUNCTION:
		return constructPrologTerm(
			(ConnectiveFormula*)phi.get(), semicolon_functor, pl_term);
	default:
		spdlog::warn("Ignoring unknown formula type '{}'.", (int)phi->type());
		return false;
	}
}

bool PrologQuery::constructPrologTerm(const std::shared_ptr<Term>& qa_term, term_t &pl_term)
{
	switch(qa_term->type()) {
	case TermType::VARIABLE: {
		Variable *qa_var = (Variable*)qa_term.get();
		// try to use previously created term_t
		auto it = vars_.find(qa_var->name());
		if(it != vars_.end()) {
			return PL_put_term(pl_term, it->second);
		}
		// create a new variable
		else if(PL_put_variable(pl_term)) {
			vars_[qa_var->name()] = pl_term;
			return true;
		}
		else {
			return false;
		}
	}
	case TermType::PREDICATE: {
		Predicate *qa_pred = (Predicate*)qa_term.get();
		if(qa_pred->indicator().arity()>0) {
			// create a term reference for each argument of qa_pred
			term_t pl_arg = PL_new_term_refs(qa_pred->indicator().arity());
			term_t pl_arg0 = pl_arg;
			// construct argument terms
			for(const auto &qa_arg : qa_pred->arguments()) {
				if(!constructPrologTerm(qa_arg, pl_arg)) {
					return false;
				}
				pl_arg += 1;
			}
			// construct output term
			return PL_cons_functor_v(pl_term,
				PL_new_functor(
					PL_new_atom(qa_pred->indicator().functor().c_str()),
					qa_pred->indicator().arity()),
				pl_arg0);
		}
		else {
			// 0-ary predicates are atoms
			return PL_put_atom_chars(pl_term,
				qa_pred->indicator().functor().c_str());
		}
		break;
	}
	case TermType::STRING:
		return PL_put_atom_chars(pl_term,
			((StringTerm*)qa_term.get())->value().c_str());
	case TermType::DOUBLE:
		return PL_put_float(pl_term,
			((DoubleTerm*)qa_term.get())->value());
	case TermType::INT32:
		return PL_put_integer(pl_term,
			((Integer32Term*)qa_term.get())->value());
	case TermType::LONG:
		return PL_put_integer(pl_term,
			((LongTerm*)qa_term.get())->value());
	case TermType::LIST: {
		if(!PL_put_nil(pl_term)) {
			return false;
		}
	
		ListTerm *list = (ListTerm*)qa_term.get();
		term_t pl_elem = PL_new_term_ref();
		for(auto &elem : list->elements()) {
			if(!constructPrologTerm(elem, pl_elem) ||
			   !PL_cons_list(pl_term, pl_elem, pl_term))
			{
				return false;
			}
		}
		return true;
	}
	default:
		spdlog::warn("Ignoring unknown term type '{}'.", (int)qa_term->type());
		return false;
	}
}

bool PrologQuery::constructPrologTerm(ConnectiveFormula *phi, functor_t &pl_functor, term_t &pl_term)
{
	term_t pl_pred = pl_term;
	int counter = 1;
	
	for(const auto &psi : phi->formulae()) {
		if(counter < phi->formulae().size()) {
			// create a fresh term for each argument of the operator
			term_t pl_arg = PL_new_term_refs(2);
			// construct the first argument
			if(!constructPrologTerm(psi, pl_arg)) return false;
			// construct the operator predicate (leave second argument unspecified)
			if(!PL_cons_functor_v(pl_pred, pl_functor, pl_arg)) {
				spdlog::warn("Failed to construct Prolog term for a formula.");
			}
			// second argument is the list constructed in the next step 
			pl_pred = (pl_arg+1);
		}
		else {
			// last formula
			if(!constructPrologTerm(psi, pl_pred)) return false;
		}
		counter += 1;
	}
	return true;
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
	return std::make_shared<Predicate>(functorName, arguments);
}

std::shared_ptr<Term> PrologQuery::constructTerm(const term_t &t)
{
	static atom_t ATOM_fail  = PL_new_atom("fail");
	static atom_t ATOM_false = PL_new_atom("false");
	static atom_t ATOM_true  = PL_new_atom("true");
	
	switch(PL_term_type(t)) {
	case PL_INTEGER: {
		long val=0;
		if(!PL_get_long(t, &val)) {
			spdlog::error("Failed to read long from term {}.", t);
			return BottomTerm::get();
		}
		return std::make_shared<LongTerm>(val);
	}
	case PL_FLOAT: {
		double val=0.0;
		if(!PL_get_float(t, &val)) {
			spdlog::error("Failed to read double from term {}.", t);
			return BottomTerm::get();
		}
		return std::make_shared<DoubleTerm>(val);
	}
	case PL_STRING: {
		char *s;
		if(!PL_get_chars(t, &s, CVT_ALL)) {
			spdlog::error("Failed to read string from term {}.", t);
			return BottomTerm::get();
		}
		return std::make_shared<StringTerm>(std::string(s));
	}
	case PL_ATOM: {
		atom_t atom;
		if(!PL_get_atom(t, &atom)) {
			spdlog::error("failed to read atom from term {}.", t);
			return BottomTerm::get();
		}
		
		// TODO: maybe below rather needs to be handled in the predicate case?
		//   not sure if predicates with arity 0 generally appear here as atoms...
		// map `fail/0` and `false/0` to BottomTerm
		if(atom == ATOM_fail || atom == ATOM_false) {
			return BottomTerm::get();
		}
		// map `true/0` to TopTerm
		else if(atom == ATOM_true) {
			return TopTerm::get();
		}
		
		return std::make_shared<StringTerm>(std::string(PL_atom_chars(atom)));
	}
	case PL_VARIABLE: {
		// variables are converted to print-name
		char *s;
		if(!PL_get_chars(t, &s, CVT_VARIABLE)) {
			spdlog::error("Failed to read variable name from term {}.", t);
			return BottomTerm::get();
		}
		return std::make_shared<Variable>(std::string(s));
	}
	case PL_NIL:
		return ListTerm::nil();
	case PL_LIST_PAIR: {
		term_t head = PL_new_term_ref();
		std::list<std::shared_ptr<Term>> elements;
		while(PL_get_list(t, head, t)) {
			elements.push_back(PrologQuery::constructTerm(head));
		}
		return std::make_shared<ListTerm>(
			std::vector<std::shared_ptr<Term>>(elements.begin(), elements.end()));
	}
	case PL_TERM:
		return PrologQuery::constructPredicate(t);
	default:
		spdlog::error("unknown term type {}.", PL_term_type(t));
		return BottomTerm::get();
	}
}

std::shared_ptr<Formula> PrologQuery::toFormula1(const std::shared_ptr<Predicate> &p)
{
	static std::string comma_functor = ",";
	static std::string semicolon_functor = ";";
	
	// special handling for "," and ";" functors
	if(p->indicator().functor() == comma_functor) {
		std::vector<std::shared_ptr<Formula>> formulae(p->indicator().arity());
		for(int i=0; i<formulae.size(); i++) {
			formulae[i] = toFormula(p->arguments()[i]);
		}
		return std::shared_ptr<Formula>(new ConjunctionFormula(formulae));
	}
	else if(p->indicator().functor() == semicolon_functor) {
		std::vector<std::shared_ptr<Formula>> formulae(p->indicator().arity());
		for(int i=0; i<formulae.size(); i++) {
			formulae[i] = toFormula(p->arguments()[i]);
		}
		return std::shared_ptr<Formula>(new DisjunctionFormula(formulae));
	}
	// use predicate as formula
	else {
		return std::shared_ptr<Formula>(new PredicateFormula(p));
	}
}

std::shared_ptr<Formula> PrologQuery::toFormula(const std::shared_ptr<Term> &t)
{
	switch(t->type()) {
	case TermType::PREDICATE: {
		return toFormula1(std::static_pointer_cast<Predicate>(t));
	}
	case TermType::VARIABLE:
	case TermType::LONG:
	case TermType::INT32:
	case TermType::DOUBLE:
	case TermType::STRING:
	case TermType::LIST:
		spdlog::warn("toFormula called with wrong term type");
		return toFormula1(BottomTerm::get());
	default:
		spdlog::warn("Ignoring unknown term type '{}'.", (int)t->type());
		return toFormula1(BottomTerm::get());
	}
}

std::shared_ptr<Query> PrologQuery::toQuery(const std::shared_ptr<Term> &t)
{
	return std::shared_ptr<Query>(new Query(toFormula(t)));
}

