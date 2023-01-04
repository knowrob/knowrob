/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/prolog/PrologQuery.h>

using namespace knowrob;

	
PrologQuery::PrologQuery(const std::shared_ptr<Query> &qa_query, const char* module)
: qa_query_(qa_query),
  pl_query_(PL_new_term_ref())
{
	// translate into term_t
	if(!constructPrologTerm(qa_query->formula(), pl_query_)) {
		throw QueryError("Failed to create term_t from Query.");
	}
	
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
			qa_pred->indicator().functor().c_str(), arity, module);
		break;
	}
	case FormulaType::CONJUNCTION:
		pl_predicate_ = PrologQuery::PREDICATE_comma();
		arity = 2;
		break;
	case FormulaType::DISJUNCTION:
		pl_predicate_ = PrologQuery::PREDICATE_semicolon();
		arity = 2;
		break;
	default:
		KB_WARN("Ignoring unknown formula type '{}'.", (int)qa_query_->formula()->type());
		break;
	}

	// create pl_arguments_
  	pl_arguments_ = PL_new_term_refs(arity);
  	for(uint32_t i=0; i<arity; ++i) {
  		term_t arg = pl_arguments_ + i;
		if(!PL_get_arg(i+1, pl_query_, arg)) {
			KB_WARN("failed to get query argument term");
		}
  	}
}

PrologQuery::~PrologQuery()
{
	PL_reset_term_refs(pl_query_);
}

bool PrologQuery::constructPrologTerm(const FormulaPtr& phi, term_t &pl_term)
{
	switch(phi->type()) {
	case FormulaType::PREDICATE: {
		const std::shared_ptr<Predicate> &qa_pred =
			((PredicateFormula*) phi.get())->predicate();
		return constructPrologTerm(qa_pred, pl_term);
	}
	case FormulaType::CONJUNCTION:
		return constructPrologTerm(
			(ConnectiveFormula*)phi.get(), PrologQuery::FUNCTOR_comma(), pl_term);
	case FormulaType::DISJUNCTION:
		return constructPrologTerm(
			(ConnectiveFormula*)phi.get(), PrologQuery::FUNCTOR_semicolon(), pl_term);
	default:
		KB_WARN("Ignoring unknown formula type '{}'.", (int)phi->type());
		return false;
	}
}

bool PrologQuery::constructPrologTerm(const TermPtr& qa_term, term_t &pl_term)
{
	switch(qa_term->type()) {
	case TermType::PREDICATE: {
		auto *qa_pred = (Predicate*)qa_term.get();
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
			// TODO: caching result of PL_new_functor() could be a good idea
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
	}
	case TermType::VARIABLE: {
		auto *qa_var = (Variable*)qa_term.get();
		// try to use previously created term_t
		auto it = vars_.find(qa_var->name());
		if(it != vars_.end()) {
			return PL_put_term(pl_term, it->second);
		}
		// create a new variable
		// TODO: any way to assign the name here?
		else if(PL_put_variable(pl_term)) {
			vars_[qa_var->name()] = pl_term;
			return true;
		}
		else {
			return false;
		}
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
		if(!PL_put_nil(pl_term)) return false;
		auto *list = (ListTerm*)qa_term.get();
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
		KB_WARN("Ignoring unknown term type '{}'.", (int)qa_term->type());
		return false;
	}
}

bool PrologQuery::constructPrologTerm(ConnectiveFormula *phi, const functor_t &pl_functor, term_t &pl_term)
{
	if(phi->formulae().size()==1) {
		return constructPrologTerm(phi->formulae()[0], pl_term);
	}
	else {
		int counter = 1;
		term_t last_head = PL_new_term_ref();
		
		for(int i=phi->formulae().size()-1; i>=0; --i) {
			if(counter==1) {
				// create term for last formula, remember in last_head term
				if(!constructPrologTerm(phi->formulae()[i], last_head)) return false;
			}
			else {
				// create a 2-ary predicate using last_head as second argument
				term_t pl_arg = PL_new_term_refs(2);
				if(!constructPrologTerm(phi->formulae()[i], pl_arg) ||
				   !PL_put_term(pl_arg+1, last_head) ||
				   !PL_cons_functor_v((i==0 ? pl_term : last_head), pl_functor, pl_arg))
				{
					return false;
				}
				
			}
			counter += 1;
		}
		
		return true;
	}
}

TermPtr PrologQuery::constructTerm(const term_t &t)
{
	switch(PL_term_type(t)) {
	case PL_TERM: {
		size_t arity;
		atom_t name;
		if(!PL_get_name_arity(t, &name, &arity)) break;
		
		std::string functorName(PL_atom_chars(name));
		// construct arguments
		std::vector<TermPtr> arguments(arity);
		term_t arg = PL_new_term_ref();
		for(int n=1; n<=arity; n++) {
			if(PL_get_arg(n, t, arg)) {
				arguments[n-1] = constructTerm(arg);
			}
			else {
				KB_WARN("Failed to construct argument {} of predicate {}.", n, functorName);
			}
		}
		// construct Predicate object
		return std::make_shared<Predicate>(functorName, arguments);
	}
	case PL_VARIABLE: {
		// TODO: could reuse existing variables here.
		// TODO: could support a mapping to internal names here
		char *s;
		if(!PL_get_chars(t, &s, CVT_VARIABLE)) break;
		return std::make_shared<Variable>(std::string(s));
	}
	case PL_ATOM: {
		atom_t atom;
		if(!PL_get_atom(t, &atom)) break;
		// TODO: maybe below rather needs to be handled in the predicate case?
		//   not sure if predicates with arity 0 generally appear here as atoms...
		// map `fail/0` and `false/0` to BottomTerm
		if(atom == PrologQuery::ATOM_fail() || atom == PrologQuery::ATOM_false()) {
			return BottomTerm::get();
		}
		// map `true/0` to TopTerm
		else if(atom == PrologQuery::ATOM_true()) {
			return TopTerm::get();
		}
		else {
			return std::make_shared<StringTerm>(std::string(PL_atom_chars(atom)));
		}
	}
	case PL_INTEGER: {
		long val=0;
		if(!PL_get_long(t, &val)) break;
		return std::make_shared<LongTerm>(val);
	}
	case PL_FLOAT: {
		double val=0.0;
		if(!PL_get_float(t, &val)) break;
		return std::make_shared<DoubleTerm>(val);
	}
	case PL_STRING: {
		char *s;
		if(!PL_get_chars(t, &s, CVT_ALL)) break;
		return std::make_shared<StringTerm>(std::string(s));
	}
	case PL_NIL:
		return ListTerm::nil();
	case PL_LIST_PAIR: {
		term_t head = PL_new_term_ref();
		std::list<TermPtr> elements;
		while(PL_get_list(t, head, t)) {
			elements.push_back(PrologQuery::constructTerm(head));
		}
		return std::make_shared<ListTerm>(
			std::vector<TermPtr>(elements.begin(), elements.end()));
	}
	default:
		KB_WARN("Unknown Prolog term type {}.", PL_term_type(t));
		break;
	}
	
	KB_WARN("Failed to read Prolog term of type {}.", PL_term_type(t));
	return BottomTerm::get();
}

FormulaPtr PrologQuery::toFormula(const TermPtr &t)
{
	static std::string comma_functor     = ",";
	static std::string semicolon_functor = ";";
	
	std::shared_ptr<Predicate> p = (
		t->type()==TermType::PREDICATE ?
		std::static_pointer_cast<Predicate>(t) :
		BottomTerm::get());
	
	if(p->indicator().functor() == comma_functor) {
		std::vector<FormulaPtr> formulas(p->indicator().arity());
		for(int i=0; i<formulas.size(); i++) {
			formulas[i] = toFormula(p->arguments()[i]);
		}
		return std::make_shared<ConjunctionFormula>(formulas);
	}
	else if(p->indicator().functor() == semicolon_functor) {
		std::vector<FormulaPtr> formulas(p->indicator().arity());
		for(int i=0; i<formulas.size(); i++) {
			formulas[i] = toFormula(p->arguments()[i]);
		}
		return std::make_shared<DisjunctionFormula>(formulas);
	}
	else {
		return std::make_shared<PredicateFormula>(p);
	}
}

std::shared_ptr<Query> PrologQuery::toQuery(const std::shared_ptr<Term> &t)
{
	return std::make_shared<Query>(toFormula(t));
}

TermPtr PrologQuery::toTerm(const FormulaPtr &phi)
{
	static const auto commaIndicator     = std::make_shared<PredicateIndicator>(",",2);
	static const auto semicolonIndicator = std::make_shared<PredicateIndicator>(";",2);
	
	switch(phi->type()) {
	case FormulaType::PREDICATE:
		return ((PredicateFormula*) phi.get())->predicate();
		
	case FormulaType::CONJUNCTION:
		return toTerm((ConnectiveFormula*)phi.get(), commaIndicator);
		
	case FormulaType::DISJUNCTION:
		return toTerm((ConnectiveFormula*)phi.get(), semicolonIndicator);
		
	default:
		KB_WARN("Ignoring unknown formula type '{}'.", (int)phi->type());
		return BottomTerm::get();
	}
}

TermPtr PrologQuery::toTerm(ConnectiveFormula *phi, const std::shared_ptr<PredicateIndicator> &indicator)
{
	std::vector<TermPtr> args(2);
	int numArgs=0;
		
	for(int i=phi->formulae().size(); i>0; --i) {
		// convert next formula to term
		args[numArgs>1 ? 0 : numArgs] = toTerm(phi->formulae()[i-1]);
		// construct a predictae
		if(numArgs>0) {
			args[1] = std::make_shared<Predicate>(indicator, args);
		}
		numArgs += 1;
	}
	
	// outermost predicate is stored in args vector.
	return args[1];
}

/******************************************/
/*********** static constants *************/
/******************************************/

const atom_t& PrologQuery::ATOM_fail() {
	static atom_t a = PL_new_atom("fail");
	return a;
}

const atom_t& PrologQuery::ATOM_false() {
	static atom_t a = PL_new_atom("false");
	return a;
}

const atom_t& PrologQuery::ATOM_true() {
	static atom_t a = PL_new_atom("true");
	return a;
}

const atom_t& PrologQuery::ATOM_comma() {
	static atom_t a = PL_new_atom(",");
	return a;
}

const atom_t& PrologQuery::ATOM_semicolon() {
	static atom_t a = PL_new_atom(";");
	return a;
}

const functor_t& PrologQuery::FUNCTOR_comma() {
	static atom_t a = PL_new_functor(PrologQuery::ATOM_comma(), 2);
	return a;
}

const functor_t& PrologQuery::FUNCTOR_semicolon() {
	static atom_t a = PL_new_functor(PrologQuery::ATOM_semicolon(), 2);
	return a;
}

const predicate_t& PrologQuery::PREDICATE_comma() {
	static predicate_t a = PL_pred(PrologQuery::FUNCTOR_comma(), nullptr);
	return a;
}

const predicate_t& PrologQuery::PREDICATE_semicolon() {
	static predicate_t a = PL_pred(PrologQuery::FUNCTOR_semicolon(), nullptr);
	return a;
}

