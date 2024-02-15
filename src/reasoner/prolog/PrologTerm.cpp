/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <list>
#include <stack>
#include "knowrob/reasoner/prolog/PrologTerm.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/Logger.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/semweb/xsd.h"

using namespace knowrob;

namespace knowrob {
	static const auto triple_f = "triple";
}

// NOTE: Prolog takes care of freeing the term references when returning from C++
//       context to Prolog context. Therefore, we do not need to free the term here.
//       Only for temporary terms that are not needed in Prolog context
//       PL_reset_term_refs could be called, but it is not necessary.
// TODO: term_t cannot be used in different threads as is. For now PrologTerm objects
//       should be constructed in the same thread where the query is issued which is suboptimal.
//       One solution could be to record a functional expression in the main thread and
//       construct the term_t in the worker thread based on this expression.

PrologList::PrologList(const std::vector<PrologTerm> &elements)
		: PrologTerm() {
	PL_put_nil(plTerm_); // initialize as empty list
	for (auto &elem: elements) {
		if (!PL_cons_list(plTerm_, elem(), plTerm_)) {
			throw QueryError("Failed to put element into PrologList.");
		}
		vars_.insert(elem.vars_.begin(), elem.vars_.end());
	}
}

PrologTerm::PrologTerm()
		: plTerm_(PL_new_term_ref()) {
	// the new term reference is initialized to a variable
	vars_[getVarName(plTerm_)] = plTerm_;
}

PrologTerm::PrologTerm(const StatementData &triple, std::string_view functor)
		: plTerm_(PL_new_term_ref()) {
	putTriple(functor, triple);
}

PrologTerm::PrologTerm(const RDFLiteral &literal)
		: PrologTerm(triple_f, literal.subjectTerm(), literal.propertyTerm(), literal.objectTerm()) {
	// TODO: add support for object operator. Idea would be to use operator as unary functor for obj term.
	//       but will need operator support in sw_triple/4 predicate.
}

PrologTerm::PrologTerm(const std::vector<PrologTerm> &args, std::string_view functor)
		: plTerm_(PL_new_term_ref()) {
	// construct argument terms
	term_t plArgs = PL_new_term_refs(args.size());
	for (uint32_t i = 0; i < args.size(); i++) {
		auto &arg_i = args[i];
		if (!PL_put_term(plArgs + i, arg_i())) {
			PL_reset_term_refs(plArgs);
			throw QueryError("Failed to put argument {} into PrologTerm.", i);
		}
		vars_.insert(arg_i.vars_.begin(), arg_i.vars_.end());
	}
	// construct compound term
	if (!PL_cons_functor_v(plTerm_,
						   PL_new_functor(PL_new_atom(functor.data()), args.size()),
						   plArgs)) {
		PL_reset_term_refs(plArgs);
		throw QueryError("Failed to put functor into PrologTerm.");
	}
}

PrologTerm::PrologTerm(const PrologTerm &other)
		: plTerm_(other.plTerm_), vars_(other.vars_) {
}

PrologTerm::PrologTerm(const FormulaPtr &kbFormula)
		: plTerm_(PL_new_term_ref()) {
	putFormula(kbFormula);
}

PrologTerm::PrologTerm(const TermPtr &kbTerm)
		: plTerm_(PL_new_term_ref()) {
	putTerm(kbTerm);
}

PrologTerm PrologTerm::operator&(const PrologTerm &other) const {
	static const auto comma_f = ",";
	if (PL_term_type(plTerm_) == PL_VARIABLE) {
		return other;
	} else if (PL_term_type(other.plTerm_) == PL_VARIABLE) {
		return *this;
	} else {
		return PrologTerm(comma_f, *this, other);
	}
}

qid_t PrologTerm::openQuery(int flags) const {
	if (PL_term_type(plTerm_) != PL_TERM) {
		throw QueryError("PrologTerm is not a compound term (actual type: {}).",
						 (void *) this, plTerm_, PL_term_type(plTerm_));
	}
	// Note that the context module only matters for meta-predicates
	module_t ctxModule = module_.has_value() ?
						 PL_new_module(PL_new_atom(module_.value().data())) : nullptr;
	// get name and arity of the Prolog term
	size_t arity;
	atom_t name_atom;
	if (!PL_get_name_arity(plTerm_, &name_atom, &arity)) {
		throw QueryError("Failed to get name and arity of Prolog term.");
	}
	// create argument term_t references
	term_t args = PL_new_term_refs(arity);
	for (int i = 0; i < arity; i++) {
		if (!PL_get_arg(i + 1, plTerm_, args + i)) {
			throw QueryError("Failed to get argument {} of Prolog term.", i);
		}
	}
	// create a predicate_t object and open query
	auto pred = PL_predicate(
			PL_atom_nchars(name_atom, nullptr),
			static_cast<int>(arity),
			nullptr);
	return PL_open_query(ctxModule, flags, pred, args);
}

bool PrologTerm::nextSolution(qid_t qid) const {
	if (PL_next_solution(qid)) {
		return true;
	} else {
		auto pl_exception = PL_exception(qid);
		if (pl_exception != (term_t) 0) {
			// there was an exception
			auto errorTerm = PrologTerm::toKnowRobTerm(pl_exception);
			PL_clear_exception();
			PL_close_query(qid);
			throw QueryError("Prolog error: {}.", *errorTerm);
		}
		// no exception
		return false;
	}
}

#ifdef KNOWROB_USE_PROLOG_RDF11
static inline bool putTypedLiteral(term_t pl_arg, const char *value, const std::string &xsdType) {
	// with semweb/rdf11, literals are represented as compound terms of the form `Value^^Type`
	static functor_t literalFunctor = PL_new_functor(PL_new_atom("^^"), 2);
	term_t typedLiteral = PL_new_term_refs(2);
	return PL_put_atom_chars(typedLiteral, value) &&
		   PL_put_atom_chars(typedLiteral + 1, xsdType.c_str()) &&
		   PL_cons_functor_v(pl_arg, literalFunctor, typedLiteral);
}
#else

static inline bool putTypedLiteral(term_t pl_arg, const char *value, const std::string &xsdType) {
	// with semweb/rdf_db, literals are represented as compound terms of the form `literal(type(Type,Value))`
	static functor_t literalFunctor = PL_new_functor(PL_new_atom("literal"), 1);
	static functor_t literalTypeFunctor = PL_new_functor(PL_new_atom("type"), 2);
	term_t typedLiteral = PL_new_term_refs(3);
	return PL_put_atom_chars(typedLiteral, xsdType.c_str()) &&
		   PL_put_atom_chars(typedLiteral + 1, value) &&
		   PL_cons_functor_v(typedLiteral + 2, literalTypeFunctor, typedLiteral) &&
		   PL_cons_functor_v(pl_arg, literalFunctor, typedLiteral + 2);
}

#endif

bool PrologTerm::putTriple(std::string_view functor, const StatementData &triple) {
	term_t pl_arg = PL_new_term_refs(4);
	if (!PL_put_atom_chars(pl_arg, triple.subject) ||
		!PL_put_atom_chars(pl_arg + 1, triple.predicate)) {
		PL_reset_term_refs(pl_arg);
		return false;
	}
	switch (triple.objectType) {
		case RDF_RESOURCE:
		case RDF_STRING_LITERAL:
			if (!PL_put_atom_chars(pl_arg + 2, triple.object)) {
				PL_reset_term_refs(pl_arg);
				return false;
			}
			break;
		case RDF_INT64_LITERAL: {
			if (!putTypedLiteral(pl_arg + 2, triple.object, xsd::IRI_integer)) {
				PL_reset_term_refs(pl_arg);
				return false;
			}
			break;
		}
		case RDF_DOUBLE_LITERAL: {
			if (!putTypedLiteral(pl_arg + 2, triple.object, xsd::IRI_double)) {
				PL_reset_term_refs(pl_arg);
				return false;
			}
			break;
		}
		case RDF_BOOLEAN_LITERAL: {
			if (!putTypedLiteral(pl_arg + 2, triple.object, xsd::IRI_boolean)) {
				PL_reset_term_refs(pl_arg);
				return false;
			}
			break;
		}
	}

	if (triple.graph) {
		if (!PL_put_atom_chars(pl_arg + 3, triple.graph)) {
			PL_reset_term_refs(pl_arg);
			return false;
		}
	} else {
		static const auto fallbackOrigin = "user";
		KB_WARN("triple has no graph, using fallback origin '{}'.", fallbackOrigin);
		if (!PL_put_atom_chars(pl_arg + 3, fallbackOrigin)) {
			PL_reset_term_refs(pl_arg);
			return false;
		}
	}

	auto pl_functor = PL_new_functor(PL_new_atom(functor.data()), 4);
	if (!PL_cons_functor_v(plTerm_, pl_functor, pl_arg)) {
		PL_reset_term_refs(pl_arg);
		throw QueryError("Failed to put triple into PrologTerm.");
	}
	vars_.clear();

	return true;
}

bool PrologTerm::putFormula(const FormulaPtr &phi) {
	// reset vars before putting a new formula
	vars_.clear();
	return putFormula(phi, plTerm_);
}

bool PrologTerm::putFormula(const FormulaPtr &phi, term_t plTerm) { //NOLINT
	static auto negationFun = PL_new_functor(PL_new_atom("\\+"), 1);
	static auto implicationFun = PL_new_functor(PL_new_atom(":-"), 2);

	switch (phi->type()) {
		case FormulaType::PREDICATE: {
			auto qa_pred = std::static_pointer_cast<Predicate>(phi);
			auto qa_term = std::static_pointer_cast<Term>(qa_pred);
			return putTerm(qa_term, plTerm);
		}

		case FormulaType::NEGATION: {
			auto negated = std::static_pointer_cast<Negation>(phi)->negatedFormula();
			auto negated_t = PL_new_term_ref();
			return putFormula(negated, negated_t) &&
				   PL_cons_functor_v(plTerm, negationFun, negated_t);
		}

		case FormulaType::IMPLICATION: {
			auto implication = std::static_pointer_cast<Implication>(phi);
			auto args = PL_new_term_refs(2);
			return putFormula(implication->consequent(), args) &&
				   putFormula(implication->antecedent(), args + 1) &&
				   PL_cons_functor_v(plTerm, implicationFun, args);
		}

		case FormulaType::CONJUNCTION:
			return putCompound((CompoundFormula *) phi.get(), plTerm, PrologTerm::FUNCTOR_comma());

		case FormulaType::DISJUNCTION:
			return putCompound((CompoundFormula *) phi.get(), plTerm, PrologTerm::FUNCTOR_semicolon());

		case FormulaType::MODAL:
			KB_WARN("Modal formula cannot be mapped to Prolog terms.");
			return false;
	}
	return false;
}

bool PrologTerm::putTerm(const TermPtr &kbTerm) {
	// reset vars before putting a new term
	vars_.clear();
	return putTerm(kbTerm, plTerm_);
}

bool PrologTerm::putTerm(const TermPtr &kbTerm, term_t pl_term) { //NOLINT
	switch (kbTerm->type()) {
		case TermType::PREDICATE: {
			auto *qa_pred = (Predicate *) kbTerm.get();
			if (qa_pred->indicator()->arity() > 0) {
				// create a term reference for each argument of qa_pred
				term_t pl_arg = PL_new_term_refs(qa_pred->indicator()->arity());
				term_t pl_arg0 = pl_arg;
				// construct argument terms
				for (const auto &qa_arg: qa_pred->arguments()) {
					if (!putTerm(qa_arg, pl_arg)) {
						return false;
					}
					pl_arg += 1;
				}
				// construct output term
				// TODO: caching result of PL_new_functor() could be a good idea
				return PL_cons_functor_v(pl_term,
										 PL_new_functor(
												 PL_new_atom(qa_pred->indicator()->functor().c_str()),
												 qa_pred->indicator()->arity()),
										 pl_arg0);
			} else {
				// 0-ary predicates are atoms
				return PL_put_atom_chars(pl_term,
										 qa_pred->indicator()->functor().c_str());
			}
		}
		case TermType::VARIABLE: {
			auto *qa_var = (Variable *) kbTerm.get();
			// try to use previously created term_t
			auto it = vars_.find(qa_var->name());
			if (it != vars_.end()) {
				return PL_put_term(pl_term, it->second);
			}
				// create a new variable
			else if (PL_put_variable(pl_term)) {
				vars_[qa_var->name()] = pl_term;
				return true;
			} else {
				return false;
			}
		}
		case TermType::STRING:
			return PL_put_atom_chars(pl_term,
									 ((StringTerm *) kbTerm.get())->value().c_str());
		case TermType::DOUBLE:
			return PL_put_float(pl_term,
								((DoubleTerm *) kbTerm.get())->value());
		case TermType::INT32:
			return PL_put_integer(pl_term,
								  ((Integer32Term *) kbTerm.get())->value());
		case TermType::LONG:
			return PL_put_integer(pl_term,
								  ((LongTerm *) kbTerm.get())->value());
		case TermType::LIST: {
			if (!PL_put_nil(pl_term)) return false;
			auto *list = (ListTerm *) kbTerm.get();
			term_t pl_elem = PL_new_term_ref();
			for (auto &elem: list->elements()) {
				if (!putTerm(elem, pl_elem) ||
					!PL_cons_list(pl_term, pl_elem, pl_term)) {
					return false;
				}
			}
			return true;
		}
	}

	return false;
}

bool PrologTerm::putCompound(CompoundFormula *phi, term_t pl_term, const functor_t &pl_functor) { //NOLINT
	if (phi->formulae().size() == 1) {
		return putFormula(phi->formulae()[0], pl_term);
	} else {
		int counter = 1;
		term_t last_head = PL_new_term_ref();

		for (auto i = phi->formulae().size() - 1; i >= 0; --i) {
			if (counter == 1) {
				// create term for last formula, remember in last_head term
				if (!putFormula(phi->formulae()[i], last_head)) return false;
			} else {
				// create a 2-ary predicate using last_head as second argument
				term_t pl_arg = PL_new_term_refs(2);
				if (!putFormula(phi->formulae()[i], pl_arg) ||
					!PL_put_term(pl_arg + 1, last_head) ||
					!PL_cons_functor_v((i == 0 ? pl_term : last_head), pl_functor, pl_arg)) {
					return false;
				}

			}
			counter += 1;
		}

		return true;
	}
}

std::string PrologTerm::getVarName(term_t plTerm) {
	char *s;
	if (PL_get_chars(plTerm, &s, CVT_VARIABLE)) {
		return s;
	} else {
		throw QueryError("Failed to get variable name.");
	}
}

void PrologTerm::readVars(term_t plTerm) {
	std::stack<term_t> stack;
	stack.push(plTerm);

	while (!stack.empty()) {
		term_t t = stack.top();
		stack.pop();

		switch (PL_term_type(t)) {
			case PL_VARIABLE: {
				vars_[getVarName(t)] = t;
				break;
			}
			case PL_DICT: {
				// TODO: support Prolog dicts. but not sure how to iterate over them. there is just:
				//       `int PL_get_dict_key(atom_t key, term_t +dict, term_t -value)`
				//       Maybe PL_get_name_arity and PL_get_arg can be used just like in the compound case below?
				//       if this is the case both types could be handled in the same way.
				KB_WARN("PrologTerm::readVars: PL_DICT not supported.");
				break;
			}
			case PL_TERM: { // A compound term
				size_t arity;
				atom_t name;
				if (PL_get_name_arity(t, &name, &arity)) {
					for (int n = 1; n <= arity; n++) {
						term_t arg = PL_new_term_ref();
						if (PL_get_arg(n, t, arg)) {
							stack.push(arg);
						}
					}
				}
				break;
			}
			case PL_LIST_PAIR: {
				term_t head = PL_new_term_ref();
				while (PL_get_list(t, head, t)) {
					stack.push(head);
				}
				break;
			}
			default: // grounded term -> no vars
				break;
		}
	}
}

TermPtr PrologTerm::toKnowRobTerm() const {
	return toKnowRobTerm(plTerm_);
}

TermPtr PrologTerm::toKnowRobTerm(const term_t &t) { //NOLINT
	switch (PL_term_type(t)) {
		case PL_TERM: {
			size_t arity;
			atom_t name;
			if (!PL_get_name_arity(t, &name, &arity)) break;

			std::string functorName(PL_atom_chars(name));
			// construct arguments
			std::vector<TermPtr> arguments(arity);
			term_t arg = PL_new_term_ref();
			for (int n = 1; n <= arity; n++) {
				if (PL_get_arg(n, t, arg)) {
					arguments[n - 1] = toKnowRobTerm(arg);
				} else {
					KB_WARN("Failed to construct argument {} of predicate {}.", n, functorName);
				}
			}
			// construct Predicate object
			return std::make_shared<Predicate>(functorName, arguments);
		}
		case PL_VARIABLE: {
			// TODO: could reuse existing variables here.
			// TODO: could support a mapping to internal names here
			return std::make_shared<Variable>(getVarName(t));
		}
		case PL_ATOM: {
			atom_t atom;
			if (!PL_get_atom(t, &atom)) break;
			// TODO: maybe below rather needs to be handled in the predicate case?
			//   not sure if predicates with arity 0 generally appear here as atoms...
			// map `fail/0` and `false/0` to BottomTerm
			if (atom == PrologTerm::ATOM_fail() || atom == PrologTerm::ATOM_false()) {
				return Bottom::get();
			}
				// map `true/0` to TopTerm
			else if (atom == PrologTerm::ATOM_true()) {
				return Top::get();
			} else {
				return std::make_shared<StringTerm>(std::string(PL_atom_chars(atom)));
			}
		}
		case PL_INTEGER: {
			long val = 0;
			if (!PL_get_long(t, &val)) break;
			return std::make_shared<LongTerm>(val);
		}
		case PL_FLOAT: {
			double val = 0.0;
			if (!PL_get_float(t, &val)) break;
			return std::make_shared<DoubleTerm>(val);
		}
		case PL_STRING: {
			char *s;
			if (!PL_get_chars(t, &s, CVT_ALL)) break;
			return std::make_shared<StringTerm>(std::string(s));
		}
		case PL_NIL:
			return ListTerm::nil();
		case PL_LIST_PAIR: {
			term_t head = PL_new_term_ref();
			std::list<TermPtr> elements;
			while (PL_get_list(t, head, t)) {
				elements.push_back(toKnowRobTerm(head));
			}
			return std::make_shared<ListTerm>(
					std::vector<TermPtr>(elements.begin(), elements.end()));
		}
		default:
			KB_WARN("Unknown Prolog term type {}.", PL_term_type(t));
			break;
	}

	KB_WARN("Failed to read Prolog term of type {}.", PL_term_type(t));
	return Bottom::get();
}

FormulaPtr PrologTerm::toKnowRobFormula() const {
	return toKnowRobFormula(toKnowRobTerm());
}

FormulaPtr PrologTerm::toKnowRobFormula(const TermPtr &t) //NOLINT
{
	static std::string comma_functor = ",";
	static std::string semicolon_functor = ";";

	std::shared_ptr<Predicate> p = (
			t->type() == TermType::PREDICATE ?
			std::static_pointer_cast<Predicate>(t) :
			Bottom::get());

	if (p->indicator()->functor() == comma_functor) {
		std::vector<FormulaPtr> formulas(p->indicator()->arity());
		for (int i = 0; i < formulas.size(); i++) {
			formulas[i] = toKnowRobFormula(p->arguments()[i]);
		}
		return std::make_shared<Conjunction>(formulas);
	} else if (p->indicator()->functor() == semicolon_functor) {
		std::vector<FormulaPtr> formulas(p->indicator()->arity());
		for (int i = 0; i < formulas.size(); i++) {
			formulas[i] = toKnowRobFormula(p->arguments()[i]);
		}
		return std::make_shared<Disjunction>(formulas);
	} else {
		return p;
	}
}

/******************************************/
/*********** static constants *************/
/******************************************/

const atom_t &PrologTerm::ATOM_fail() {
	static atom_t a = PL_new_atom("fail");
	return a;
}

const atom_t &PrologTerm::ATOM_false() {
	static atom_t a = PL_new_atom("false");
	return a;
}

const atom_t &PrologTerm::ATOM_true() {
	static atom_t a = PL_new_atom("true");
	return a;
}

const atom_t &PrologTerm::ATOM_comma() {
	static atom_t a = PL_new_atom(",");
	return a;
}

const atom_t &PrologTerm::ATOM_semicolon() {
	static atom_t a = PL_new_atom(";");
	return a;
}

const functor_t &PrologTerm::FUNCTOR_comma() {
	static atom_t a = PL_new_functor(PrologTerm::ATOM_comma(), 2);
	return a;
}

const functor_t &PrologTerm::FUNCTOR_semicolon() {
	static atom_t a = PL_new_functor(PrologTerm::ATOM_semicolon(), 2);
	return a;
}

const predicate_t &PrologTerm::PREDICATE_comma() {
	static predicate_t a = PL_pred(PrologTerm::FUNCTOR_comma(), nullptr);
	return a;
}

const predicate_t &PrologTerm::PREDICATE_semicolon() {
	static predicate_t a = PL_pred(PrologTerm::FUNCTOR_semicolon(), nullptr);
	return a;
}
