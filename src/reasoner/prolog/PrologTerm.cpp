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
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/triples/GraphQuery.h"
#include "knowrob/triples/GraphSequence.h"

using namespace knowrob;

namespace knowrob {
	static const auto triple_f = "triple";
}

// NOTE: Prolog takes care of freeing the term references when returning from C++
//       context to Prolog context. Therefore, we do not need to free the term here.
//       Only for temporary terms that are not needed in Prolog context
//       PL_reset_term_refs could be called, but it is not necessary.
// NOTE: term_t cannot be used in different threads as is. For now PrologTerm objects
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
		unifyVars(elem);
	}
}

PrologTerm::PrologTerm()
		: plTerm_(PL_new_term_ref()) {
	// the new term reference is initialized to a variable
	vars_[getVarName(plTerm_)] = plTerm_;
}

PrologTerm::PrologTerm(const FramedTriple &triple, std::string_view functor)
		: plTerm_(PL_new_term_ref()) {
	putTriple(functor, triple);
}

PrologTerm::PrologTerm(const std::shared_ptr<GraphTerm> &term) //NOLINT(misc-no-recursion)
		: plTerm_(PL_new_term_ref()) {
	putTerm(term);
}

PrologTerm::PrologTerm(const GraphQueryPtr &query)
		: plTerm_(PL_new_term_ref()) {
	putTerm(query->term());
}

PrologTerm::PrologTerm(const FramedTriplePattern &pattern, const char *functor)
		: plTerm_(PL_new_term_ref()) {
	putTriplePattern(pattern, functor, plTerm_);
}

PrologTerm::PrologTerm(const std::vector<PrologTerm> &args, std::string_view functor)
		: plTerm_(PL_new_term_ref()) {
	putCompoundTerm(plTerm_, functor, args);
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

PrologTerm PrologTerm::nil() {
	PrologTerm empty_list;
	PL_put_nil(empty_list());
	return empty_list;
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

PrologTerm PrologTerm::operator|(const PrologTerm &other) const {
	static const auto semicolon_f = ";";
	if (PL_term_type(plTerm_) == PL_VARIABLE) {
		return other;
	} else if (PL_term_type(other.plTerm_) == PL_VARIABLE) {
		return *this;
	} else {
		return PrologTerm(semicolon_f, *this, other);
	}
}

PrologTerm PrologTerm::operator~() const {
	static const auto semicolon_f = "\\+";
	return PrologTerm(semicolon_f, *this);
}

void PrologTerm::unifyVars(const PrologTerm &other) {
	for (auto &pair : other.vars_) {
		if (vars_.find(pair.first) == vars_.end()) {
			vars_[pair.first] = pair.second;
		} else {
			if(!PL_unify(vars_[pair.first], pair.second)) {
				KB_WARN("failed to unify two variables.");
			}
		}
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

bool PrologTerm::nextSolution(qid_t qid) {
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
#endif

#ifdef KNOWROB_USE_PROLOG_RDF_DB

static inline bool putTypedLiteral(term_t pl_arg, std::string_view value, std::string_view xsdType) {
	// with semweb/rdf_db, literals are represented as compound terms of the form `literal(type(Type,Value))`
	static functor_t literalFunctor = PL_new_functor(PL_new_atom("literal"), 1);
	static functor_t literalTypeFunctor = PL_new_functor(PL_new_atom("type"), 2);
	term_t typedLiteral = PL_new_term_refs(3);
	return PL_put_atom_chars(typedLiteral, xsdType.data()) &&
		   PL_put_atom_chars(typedLiteral + 1, value.data()) &&
		   PL_cons_functor_v(typedLiteral + 2, literalTypeFunctor, typedLiteral) &&
		   PL_cons_functor_v(pl_arg, literalFunctor, typedLiteral + 2);
}

static inline bool putTyped(term_t pl_arg, std::string_view value, std::string_view xsdType) {
	// with semweb/rdf_db, literals are represented as compound terms of the form `type(Type,Value)`
	static functor_t literalTypeFunctor = PL_new_functor(PL_new_atom("type"), 2);
	term_t typedLiteral = PL_new_term_refs(2);
	return PL_put_atom_chars(typedLiteral, xsdType.data()) &&
		   PL_put_atom_chars(typedLiteral + 1, value.data()) &&
		   PL_cons_functor_v(pl_arg, literalTypeFunctor, typedLiteral);
}
#endif

bool PrologTerm::putCompoundTerm(term_t plTerm, std::string_view functor, const std::vector<PrologTerm> &args) {
	// construct argument terms
	term_t plArgs = PL_new_term_refs(args.size());
	for (uint32_t i = 0; i < args.size(); i++) {
		auto &arg_i = args[i];
		if (!PL_put_term(plArgs + i, arg_i())) {
			PL_reset_term_refs(plArgs);
			KB_WARN("Failed to put argument {} into PrologTerm.", i);
			return false;
		}
		unifyVars(arg_i);
	}
	// construct compound term
	if (!PL_cons_functor_v(plTerm,
						   PL_new_functor(PL_new_atom(functor.data()), args.size()),
						   plArgs)) {
		PL_reset_term_refs(plArgs);
		KB_WARN("Failed to put functor into PrologTerm.");
		return false;
	}
	return true;
}

bool PrologTerm::putTriple(std::string_view functor, const FramedTriple &triple) {
	term_t pl_arg = PL_new_term_refs(4);
	if (!PL_put_atom_chars(pl_arg, triple.subject().data()) ||
		!PL_put_atom_chars(pl_arg + 1, triple.predicate().data())) {
		PL_reset_term_refs(pl_arg);
		return false;
	}
	bool o_put = false;
	if (triple.isObjectIRI() || triple.isObjectBlank()) {
		o_put = PL_put_atom_chars(pl_arg + 2, triple.valueAsString().data());
	} else if (triple.xsdType()) {
		auto xsdType = xsdTypeToIRI(triple.xsdType().value());
		switch (triple.xsdType().value()) {
			case XSDType::STRING:
				o_put = putTypedLiteral(pl_arg + 2, triple.valueAsString().data(), xsdType);
				break;
			default:
				o_put = putTypedLiteral(pl_arg + 2, triple.createStringValue(), xsdType);
				break;
		}
	}
	if (!o_put) {
		PL_reset_term_refs(pl_arg);
		return false;
	}

	if (triple.graph()) {
		if (!PL_put_atom_chars(pl_arg + 3, (*triple.graph()).data())) {
			PL_reset_term_refs(pl_arg);
			return false;
		}
	} else {
		// triple has no graph, using fallback origin ImportHierarchy::ORIGIN_USER
		if (!PL_put_atom_chars(pl_arg + 3, ImportHierarchy::ORIGIN_USER.data())) {
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
		case FormulaType::PREDICATE:
			return putTerm(Predicate::toFunction(std::static_pointer_cast<Predicate>(phi)), plTerm);

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

bool PrologTerm::putTerm(const std::shared_ptr<GraphTerm> &graphTerm) { //NOLINT(misc-no-recursion)
	// reset vars before putting a new term
	vars_.clear();
	return putTerm(graphTerm, plTerm_);
}

bool PrologTerm::putFunction(Function *fn, term_t pl_term) { //NOLINT
	if (fn->arity() > 0) {
		// create a term reference for each argument of qa_pred
		term_t pl_arg = PL_new_term_refs(fn->arity());
		term_t pl_arg0 = pl_arg;
		// construct argument terms
		for (const auto &qa_arg: fn->arguments()) {
			if (!putTerm(qa_arg, pl_arg)) {
				return false;
			}
			pl_arg += 1;
		}
		// construct output term
		// TODO: caching result of PL_new_functor() could be a good idea
		return PL_cons_functor_v(pl_term,
								 PL_new_functor(
										 PL_new_atom(fn->functor()->stringForm().data()),
										 fn->arity()),
								 pl_arg0);
	} else {
		// 0-ary predicates are atoms
		return PL_put_atom_chars(pl_term,
								 fn->functor()->stringForm().data());
	}
}

bool PrologTerm::putList(ListTerm *list, term_t pl_term) { //NOLINT
	if (!PL_put_nil(pl_term)) return false;
	term_t pl_elem = PL_new_term_ref();
	for (auto &elem: list->elements()) {
		if (!putTerm(elem, pl_elem) ||
			!PL_cons_list(pl_term, pl_elem, pl_term)) {
			return false;
		}
	}
	return true;
}


bool PrologTerm::putTerm(const std::shared_ptr<GraphTerm> &kb_term, term_t pl_term) { //NOLINT(misc-no-recursion)
	static const auto rdf_has_f = "rdf_has";

	switch (kb_term->termType()) {
		case GraphTermType::Pattern:
			return putTriplePattern(*std::static_pointer_cast<GraphPattern>(kb_term)->value(), rdf_has_f, pl_term);

		case GraphTermType::Builtin:
			return putBuiltin(std::static_pointer_cast<GraphBuiltin>(kb_term), pl_term);

		case GraphTermType::Union: {
			PrologTerm goal;
			for (const auto &subTerm: std::static_pointer_cast<GraphSequence>(kb_term)->terms()) {
				goal = (goal | PrologTerm(subTerm));
			}
			unifyVars(goal);
			return PL_put_term(pl_term, goal());
		}
		case GraphTermType::Sequence: {
			PrologTerm goal;
			for (const auto &subTerm: std::static_pointer_cast<GraphSequence>(kb_term)->terms()) {
				goal = (goal & PrologTerm(subTerm));
			}
			unifyVars(goal);
			return PL_put_term(pl_term, goal());
		}
	}
	return false;
}

bool PrologTerm::putTriplePattern(const FramedTriplePattern &pat, const char *functor, term_t plTerm) {
	static const auto less_f = "lt";
	static const auto less_or_equal_f = "le";
	static const auto greater_f = "gt";
	static const auto greater_or_equal_f = "ge";
	static const auto equal_f = "eq";

	PrologTerm objectTerm, patternTerm;
	if (pat.objectOperator() == FilterType::EQ) {
		// make sure that `literal(type(Type,Value))` is used for typed literals.
		objectTerm.isRDFTerm_ = true;
		objectTerm.putTerm(pat.objectTerm());
		patternTerm = PrologTerm(functor, pat.subjectTerm(), pat.propertyTerm(), objectTerm);
		if (pat.isNegated()) {
			patternTerm = ~patternTerm;
		}
		if (pat.isOptional()) {
			patternTerm = PrologTerm("ignore", patternTerm);
		}
	} else {
		// make sure that `type(Type,Value)` is used for typed literals.
		objectTerm.isRDFFilter_ = true;
		objectTerm.putTerm(pat.objectTerm());
		bool negate = pat.isNegated();
		PrologTerm filter;
		switch (pat.objectOperator()) {
			case FilterType::EQ:
				filter = PrologTerm(equal_f, objectTerm);
				break;
			case FilterType::NEQ:
				filter = PrologTerm(equal_f, objectTerm);
				negate = !negate;
				break;
			case FilterType::LT:
				filter = PrologTerm(less_f, objectTerm);
				break;
			case FilterType::LEQ:
				filter = PrologTerm(less_or_equal_f, objectTerm);
				break;
			case FilterType::GT:
				filter = PrologTerm(greater_f, objectTerm);
				break;
			case FilterType::GEQ:
				filter = PrologTerm(greater_or_equal_f, objectTerm);
				break;
		}
		// Note: Object can be of the form literal(+Query, -Value)
		PrologTerm objectVar = (pat.objectVariable() ? PrologTerm(pat.objectVariable()) : PrologTerm());
		if (pat.objectVariable()) {
			vars_[std::string(pat.objectVariable()->name())] = objectVar();
		}
		patternTerm = PrologTerm(functor,
						         pat.subjectTerm(),
						         pat.propertyTerm(),
						         PrologTerm("literal", filter, objectVar));
		if (negate) {
			patternTerm = ~patternTerm;
		}
		if (pat.isOptional()) {
			PrologTerm negatedTerm(functor, pat.subjectTerm(), pat.propertyTerm(), PrologTerm());
			patternTerm = PrologTerm("\\+", negatedTerm) | patternTerm;
		}
	}

	unifyVars(patternTerm);
	return PL_put_term(plTerm, patternTerm());
}

bool PrologTerm::putBuiltin(const std::shared_ptr<GraphBuiltin> &builtin, term_t plTerm) {
	static const auto less_a = Atom::Tabled("<");
	static const auto less_or_equal_a = Atom::Tabled("=<");
	static const auto greater_a = Atom::Tabled(">");
	static const auto greater_or_equal_a = Atom::Tabled(">=");
	static const auto equal_a = Atom::Tabled("==");
	static const auto bind_f = "=";
	static const auto min_f = "sw_literal_min";
	static const auto max_f = "sw_literal_max";
	static const auto compare_f = "sw_literal_compare";

	switch (builtin->builtinType()) {
		case GraphBuiltinType::Less:
			if (builtin->arguments().size() == 2) {
				PrologTerm t = PrologTerm(compare_f, less_a, builtin->arguments()[0], builtin->arguments()[1]);
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::LessOrEqual:
			if (builtin->arguments().size() == 2) {
				PrologTerm t = PrologTerm(compare_f, less_or_equal_a, builtin->arguments()[0], builtin->arguments()[1]);
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::Greater:
			if (builtin->arguments().size() == 2) {
				PrologTerm t = PrologTerm(compare_f, greater_a, builtin->arguments()[0], builtin->arguments()[1]);
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::GreaterOrEqual:
			if (builtin->arguments().size() == 2) {
				PrologTerm t = PrologTerm(compare_f, greater_or_equal_a, builtin->arguments()[0], builtin->arguments()[1]);
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::Equal:
			if (builtin->arguments().size() == 2) {
				PrologTerm t = PrologTerm(compare_f, equal_a, builtin->arguments()[0], builtin->arguments()[1]);
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::Bind:
			if (builtin->arguments().size() == 1 && builtin->bindVar() != nullptr) {
				PrologTerm t(bind_f, builtin->bindVar(), builtin->arguments()[0]);
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::Min:
			if (builtin->arguments().size() == 2 && builtin->bindVar() != nullptr) {
				PrologTerm t(min_f, builtin->arguments()[0], builtin->arguments()[1], builtin->bindVar());
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
		case GraphBuiltinType::Max:
			if (builtin->arguments().size() == 2 && builtin->bindVar() != nullptr) {
				PrologTerm t(max_f, builtin->arguments()[0], builtin->arguments()[1], builtin->bindVar());
				unifyVars(t);
				return PL_put_term(plTerm, t());
			}
			break;
	}
	return false;
}

bool PrologTerm::putTerm(const TermPtr &kbTerm, term_t pl_term) { //NOLINT
	switch (kbTerm->termType()) {
		case TermType::FUNCTION: {
			auto *fn = (Function *) kbTerm.get();
			if (*fn->functor() == *ListTerm::listFunctor()) {
				return putList((ListTerm *) fn, pl_term);
			} else {
				return putFunction(fn, pl_term);
			}
		}
		case TermType::VARIABLE: {
			auto *qa_var = (Variable *) kbTerm.get();
			auto it = vars_.find(qa_var->name());
			if (it != vars_.end()) {
				// try to use previously created term_t
				return PL_put_term(pl_term, it->second);
			} else if (PL_put_variable(pl_term)) {
				// create a new variable
				// Note: it is apparently impossible to assign a name to a variable.
				//       This is why we have to store the variable name in the map, and potentially create
				//       unneeded term_t references for variables that must be unified before querying anyway.
				//       the alternative would be to carry around a shared variable map which would make this
				//       interface slightly ugly
				vars_[std::string(qa_var->name())] = pl_term;
				return true;
			} else {
				return false;
			}
		}
		case TermType::ATOMIC:
			if (isRDFTerm_ || isRDFFilter_) {
				return putRDFAtomic(std::static_pointer_cast<Atomic>(kbTerm), pl_term);
			} else {
				return putNativeAtomic(std::static_pointer_cast<Atomic>(kbTerm), pl_term);
			}
	}

	return false;
}

bool PrologTerm::putRDFAtomic(const std::shared_ptr<Atomic> &atomic, term_t pl_term) const {
	if (atomic->isIRI() || atomic->isBlank()) {
		return PL_put_atom_chars(pl_term, atomic->stringForm().data());
	} else if (atomic->isNumeric() || atomic->isString()) {
		auto xsdAtomic = std::static_pointer_cast<XSDAtomic>(atomic);
		auto xsdType = xsdTypeToIRI(xsdAtomic->xsdType());
		if (isRDFFilter_) {
			return putTyped(pl_term, atomic->stringForm(), xsdType);
		} else {
			return putTypedLiteral(pl_term, atomic->stringForm(), xsdType);
		}
	}
	return false;
}

bool PrologTerm::putNativeAtomic(const std::shared_ptr<Atomic> &atomic, term_t pl_term) {
	switch (atomic->atomicType()) {
		case AtomicType::ATOM:
			return PL_put_atom_chars(pl_term, atomic->stringForm().data());

		case AtomicType::STRING:
			return PL_put_string_chars(pl_term, atomic->stringForm().data());

		case AtomicType::NUMERIC: {
			auto *numeric = (Numeric *) atomic.get();
			switch (numeric->xsdType()) {
				case XSDType::DOUBLE:
					return PL_put_float(pl_term, numeric->asDouble());
				case XSDType::FLOAT:
					return PL_put_float(pl_term, numeric->asFloat());
				case XSDType::NON_NEGATIVE_INTEGER:
				case XSDType::INTEGER:
					return PL_put_integer(pl_term, numeric->asInteger());
				case XSDType::LONG:
					return PL_put_int64(pl_term, numeric->asLong());
				case XSDType::SHORT:
					return PL_put_integer(pl_term, numeric->asShort());
				case XSDType::UNSIGNED_LONG:
					return PL_put_uint64(pl_term, numeric->asUnsignedLong());
				case XSDType::UNSIGNED_INT:
					return PL_put_integer(pl_term, numeric->asUnsignedInteger());
				case XSDType::UNSIGNED_SHORT:
					return PL_put_integer(pl_term, numeric->asUnsignedShort());
				case XSDType::BOOLEAN:
					return PL_put_bool(pl_term, numeric->asBoolean());
				case XSDType::STRING:
				case XSDType::LAST:
					break;
			}
			break;
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

static TermPtr toKnowRobLiteral1(const term_t &type_term) {
	term_t xsd_type = PL_new_term_ref();
	term_t xsd_value = PL_new_term_ref();
	if (!PL_get_arg(1, type_term, xsd_type) || !PL_get_arg(2, type_term, xsd_value)) {
	    KB_WARN("failed to get argument of type term.");
		return {};
	}

	atom_t xsd_type_atom;
	if (!PL_get_atom(xsd_type, &xsd_type_atom)) {
		KB_WARN("Failed to get XSD type atom.");
		return {};
	}

	atom_t xsd_value_atom;
	if (!PL_get_atom(xsd_value, &xsd_value_atom)) {
		KB_WARN("Failed to get XSD value atom.");
		return {};
	}

	return XSDAtomic::create(PL_atom_chars(xsd_value_atom), PL_atom_chars(xsd_type_atom));
}

static TermPtr toKnowRobLiteral(const term_t &literal_term) {
	term_t type_term = PL_new_term_ref();
	if (!PL_get_arg(1, literal_term, type_term)) {
	    KB_WARN("failed to get argument of literal term.");
		return {};
	}
	if (PL_term_type(type_term) != PL_TERM) {
	    KB_WARN("argument of literal term must be a term.");
		return {};
	}

	size_t literal_arity;
	atom_t literal_name;
	if (!PL_get_name_arity(type_term, &literal_name, &literal_arity) ||
	    literal_arity != 2 || std::string_view(PL_atom_chars(literal_name)) != "type") {
	    KB_WARN("argument of literal term must be a 2-ary type term.");
		return {};
	}

	return toKnowRobLiteral1(type_term);
}

TermPtr PrologTerm::toKnowRobTerm(const term_t &t) { //NOLINT
	switch (PL_term_type(t)) {
		case PL_TERM: {
			size_t arity;
			atom_t name;
			if (!PL_get_name_arity(t, &name, &arity)) break;
			std::string functorName(PL_atom_chars(name));

			// Special handling for typed literals. whenever a term is of the form `literal(type(Type,Value))`
			// it is considered a typed literal, and converted to a corresponding Term type.
			if (functorName == "literal" && arity == 1) {
				auto xsdLiteral = toKnowRobLiteral(t);
				if (xsdLiteral) return xsdLiteral;
			}
			if (functorName == "type" && arity == 2) {
				auto xsdLiteral = toKnowRobLiteral1(t);
				if (xsdLiteral) return xsdLiteral;
			}

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
			return std::make_shared<Function>(functorName, arguments);
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
				return Bottom::get()->functor();
			}
				// map `true/0` to TopTerm
			else if (atom == PrologTerm::ATOM_true()) {
				return Top::get()->functor();
			} else {
				std::string_view charForm = PL_atom_chars(atom);
				// TODO: support that Prolog returns typed literals instead of guessing the type
				switch (rdfNodeTypeGuess(charForm)) {
					case RDFNodeType::BLANK:
						return Blank::Tabled(charForm);
					case RDFNodeType::IRI:
						return IRIAtom::Tabled(charForm);
					case RDFNodeType::LITERAL:
						return Atom::Tabled(charForm);
				}
			}
		}
		case PL_INTEGER: {
			long val = 0;
			if (!PL_get_long(t, &val)) break;
			return std::make_shared<Long>(val);
		}
		case PL_FLOAT: {
			double val = 0.0;
			if (!PL_get_float(t, &val)) break;
			return std::make_shared<Double>(val);
		}
		case PL_STRING: {
			char *s;
			if (!PL_get_chars(t, &s, CVT_ALL)) break;
			return std::make_shared<String>(s);
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
	return Bottom::get()->functor();
}

FormulaPtr PrologTerm::toKnowRobFormula() const {
	return toKnowRobFormula(toKnowRobTerm());
}

FormulaPtr PrologTerm::toKnowRobFormula(const TermPtr &t) //NOLINT
{
	static std::string comma_functor = ",";
	static std::string semicolon_functor = ";";

	std::shared_ptr<Predicate> p = (
			t->termType() == TermType::FUNCTION ?
			Predicate::fromFunction(std::static_pointer_cast<Function>(t)) :
			Bottom::get());

	if (p->functor()->stringForm() == comma_functor) {
		std::vector<FormulaPtr> formulas(p->arity());
		for (int i = 0; i < formulas.size(); i++) {
			formulas[i] = toKnowRobFormula(p->arguments()[i]);
		}
		return std::make_shared<Conjunction>(formulas);
	} else if (p->functor()->stringForm() == semicolon_functor) {
		std::vector<FormulaPtr> formulas(p->arity());
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

bool PrologTerm::display(std::ostream &os, term_t t) { //NOLINT(misc-no-recursion)
	int n;
	size_t len;
	char *s;

	switch (PL_term_type(t)) {
		case PL_ATOM:
		case PL_INTEGER:
		case PL_FLOAT:
			if (PL_get_chars(t, &s, CVT_ALL)) {
				os << s;
				return true;
			}
			break;
		case PL_VARIABLE:
			if (PL_get_chars(t, &s, CVT_VARIABLE)) {
				os << s;
				return true;
			}
			break;
		case PL_STRING:
			if (PL_get_string_chars(t, &s, &len)) {
				os << s;
				return true;
			}
			break;
		case PL_TERM: {
			term_t a = PL_new_term_ref();
			atom_t name;
			size_t arity;
			if (!PL_get_name_arity(t, &name, &arity)) break;
			std::string_view functor(PL_atom_chars(name));

			if (functor == ",") {
				for (n = 1; n <= arity; n++) {
					if (!PL_get_arg(n, t, a)) break;
					if (n > 1) os << ",\n";
					if (!display(os, a)) os << "_";
				}
			} else {
				os << functor << '(';
				for (n = 1; n <= arity; n++) {
					if (!PL_get_arg(n, t, a)) break;
					if (n > 1) os << ", ";
					if (!display(os, a)) os << "_";
				}
				os << ')';
			}
			return true;
		}
	}
	return false;
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::PrologTerm &t) {
		PrologTerm::display(os, t());
		return os;
	}
}
