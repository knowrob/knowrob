/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <list>
#include <boost/stacktrace.hpp>
#include "knowrob/Logger.h"
#include "knowrob/reasoner/prolog/PrologQuery.h"
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/formulas/Top.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Implication.h"

using namespace knowrob;

PrologQuery::PrologQuery(const std::shared_ptr<const Query> &qa_query)
		: qa_query_(qa_query),
		  pl_query_(PL_new_term_ref()) {
	// translate into term_t
	if (!putTerm(pl_query_, qa_query->formula(), vars_)) {
		throw QueryError("Failed to create term_t from Query.", boost::stacktrace::stacktrace());
	}
}

PrologQuery::~PrologQuery() {
	PL_reset_term_refs(pl_query_);
}

bool PrologQuery::putTerm( //NOLINT
		term_t pl_term, const FormulaPtr &phi, PrologVariableMap &vars) {
	static auto negationFun = PL_new_functor(PL_new_atom("\\+"), 1);
	static auto implicationFun = PL_new_functor(PL_new_atom(":-"), 2);

	switch(phi->type()) {
		case FormulaType::PREDICATE: {
			auto qa_pred = std::static_pointer_cast<Predicate>(phi);
			auto qa_term = std::static_pointer_cast<Term>(qa_pred);
			return putTerm(pl_term, qa_term, vars);
		}

		case FormulaType::NEGATION: {
			auto negated = std::static_pointer_cast<Negation>(phi)->negatedFormula();
			auto negated_t = PL_new_term_ref();
			return putTerm(negated_t, negated, vars) &&
				   PL_cons_functor_v(pl_term, negationFun, negated_t);
		}

		case FormulaType::IMPLICATION: {
			auto implication = std::static_pointer_cast<Implication>(phi);
			auto args = PL_new_term_refs(2);
			return putTerm(args, implication->consequent(), vars) &&
				   putTerm(args+1, implication->antecedent(), vars) &&
				   PL_cons_functor_v(pl_term, implicationFun, args);
		}

		case FormulaType::CONJUNCTION:
			return putTerm(pl_term, PrologQuery::FUNCTOR_comma(),
						   (CompoundFormula *) phi.get(), vars);

		case FormulaType::DISJUNCTION:
			return putTerm(pl_term, PrologQuery::FUNCTOR_semicolon(),
						   (CompoundFormula *) phi.get(), vars);

		case FormulaType::MODAL:
			KB_WARN("Modal formula cannot be mapped to Prolog terms.");
			return false;
	}
	return false;
}

bool PrologQuery::putTerm( //NOLINT
		term_t pl_term, const TermPtr &qa_term, PrologVariableMap &vars) {
	switch (qa_term->type()) {
		case TermType::PREDICATE: {
			auto *qa_pred = (Predicate *) qa_term.get();
			if (qa_pred->indicator()->arity() > 0) {
				// create a term reference for each argument of qa_pred
				term_t pl_arg = PL_new_term_refs(qa_pred->indicator()->arity());
				term_t pl_arg0 = pl_arg;
				// construct argument terms
				for (const auto &qa_arg: qa_pred->arguments()) {
					if (!putTerm(pl_arg, qa_arg, vars)) {
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
			auto *qa_var = (Variable *) qa_term.get();
			// try to use previously created term_t
			auto it = vars.find(qa_var->name());
			if (it != vars.end()) {
				return PL_put_term(pl_term, it->second);
			}
				// create a new variable
				// TODO: any way to assign the name here?
			else if (PL_put_variable(pl_term)) {
				vars[qa_var->name()] = pl_term;
				return true;
			} else {
				return false;
			}
		}
		case TermType::STRING:
			return PL_put_atom_chars(pl_term,
									 ((StringTerm *) qa_term.get())->value().c_str());
		case TermType::DOUBLE:
			return PL_put_float(pl_term,
								((DoubleTerm *) qa_term.get())->value());
		case TermType::INT32:
			return PL_put_integer(pl_term,
								  ((Integer32Term *) qa_term.get())->value());
		case TermType::LONG:
			return PL_put_integer(pl_term,
								  ((LongTerm *) qa_term.get())->value());
		case TermType::LIST: {
			if (!PL_put_nil(pl_term)) return false;
			auto *list = (ListTerm *) qa_term.get();
			term_t pl_elem = PL_new_term_ref();
			for (auto &elem: list->elements()) {
				if (!putTerm(pl_elem, elem, vars) ||
					!PL_cons_list(pl_term, pl_elem, pl_term)) {
					return false;
				}
			}
			return true;
		}
	}

	return false;
}

bool PrologQuery::putTerm( //NOLINT
		term_t pl_term, const functor_t &pl_functor, CompoundFormula *phi, PrologVariableMap &vars) {
	if (phi->formulae().size() == 1) {
		return putTerm(pl_term, phi->formulae()[0], vars);
	} else {
		int counter = 1;
		term_t last_head = PL_new_term_ref();

		for (int i = phi->formulae().size() - 1; i >= 0; --i) {
			if (counter == 1) {
				// create term for last formula, remember in last_head term
				if (!putTerm(last_head, phi->formulae()[i], vars)) return false;
			} else {
				// create a 2-ary predicate using last_head as second argument
				term_t pl_arg = PL_new_term_refs(2);
				if (!putTerm(pl_arg, phi->formulae()[i], vars) ||
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

TermPtr PrologQuery::constructTerm(const term_t &t, std::map<std::string, term_t> *vars) //NOLINT
{
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
					arguments[n - 1] = constructTerm(arg, vars);
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
			char *s;
			if (!PL_get_chars(t, &s, CVT_VARIABLE)) break;
			if (vars) (*vars)[std::string(s)] = t;
			return std::make_shared<Variable>(std::string(s));
		}
		case PL_ATOM: {
			atom_t atom;
			if (!PL_get_atom(t, &atom)) break;
			// TODO: maybe below rather needs to be handled in the predicate case?
			//   not sure if predicates with arity 0 generally appear here as atoms...
			// map `fail/0` and `false/0` to BottomTerm
			if (atom == PrologQuery::ATOM_fail() || atom == PrologQuery::ATOM_false()) {
				return Bottom::get();
			}
				// map `true/0` to TopTerm
			else if (atom == PrologQuery::ATOM_true()) {
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
				elements.push_back(PrologQuery::constructTerm(head, vars));
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

FormulaPtr PrologQuery::toFormula(const TermPtr &t) //NOLINT
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
			formulas[i] = toFormula(p->arguments()[i]);
		}
		return std::make_shared<Conjunction>(formulas);
	} else if (p->indicator()->functor() == semicolon_functor) {
		std::vector<FormulaPtr> formulas(p->indicator()->arity());
		for (int i = 0; i < formulas.size(); i++) {
			formulas[i] = toFormula(p->arguments()[i]);
		}
		return std::make_shared<Disjunction>(formulas);
	} else {
		return p;
	}
}

std::shared_ptr<FormulaQuery> PrologQuery::toQuery(const std::shared_ptr<Term> &t) {
	auto ctx = std::make_shared<QueryContext>(Query::defaultFlags());
	return std::make_shared<FormulaQuery>(toFormula(t), ctx);
}

TermPtr PrologQuery::toTerm(const FormulaPtr &phi) //NOLINT
{
	static const auto commaIndicator = std::make_shared<PredicateIndicator>(",", 2);
	static const auto semicolonIndicator = std::make_shared<PredicateIndicator>(";", 2);
	static const auto implicationIndicator = std::make_shared<PredicateIndicator>(":-", 2);
	static const auto negationIndicator = std::make_shared<PredicateIndicator>("\\+", 1);

	switch(phi->type()) {
		case FormulaType::PREDICATE:
			return std::dynamic_pointer_cast<Predicate>(phi);

		case FormulaType::CONJUNCTION:
			return toTerm((CompoundFormula *) phi.get(), commaIndicator);

		case FormulaType::DISJUNCTION:
			return toTerm((CompoundFormula *) phi.get(), semicolonIndicator);

		case FormulaType::NEGATION: {
			auto negated_t = toTerm(std::static_pointer_cast<Negation>(phi)->negatedFormula());
			return std::make_shared<Predicate>(negationIndicator, std::vector<TermPtr>{negated_t});
		}

		case FormulaType::IMPLICATION: {
			auto implication = std::static_pointer_cast<Implication>(phi);
			auto head_t = toTerm(implication->consequent());
			auto body_t = toTerm(implication->antecedent());
			return std::make_shared<Predicate>(implicationIndicator, std::vector<TermPtr>{head_t, body_t});
		}

		case FormulaType::MODAL:
			KB_WARN("Modal formula cannot be mapped to Prolog terms.");
			break;
	}

	return {};
}

TermPtr PrologQuery::toTerm(  //NOLINT
		CompoundFormula *phi,
		const std::shared_ptr<PredicateIndicator> &indicator) {
	std::vector<TermPtr> args(2);
	int numArgs = 0;

	for (int i = phi->formulae().size(); i > 0; --i) {
		// convert next formula to term
		args[numArgs > 1 ? 0 : numArgs] = toTerm(phi->formulae()[i - 1]);
		// construct a predictae
		if (numArgs > 0) {
			args[1] = std::make_shared<Predicate>(indicator, args);
		}
		numArgs += 1;
	}

	// outermost predicate is stored in args vector.
	return args[1];
}

template<class PointType>
static inline void readFuzzyInterval(
		term_t interval_term,
		std::optional<PointType> *minMin,
		std::optional<PointType> *minMax,
		std::optional<PointType> *maxMin,
		std::optional<PointType> *maxMax) {
	static const auto min_key = PL_new_atom("min");
	static const auto max_key = PL_new_atom("max");

	auto time_range = PL_new_term_ref();
	auto inner_arg = PL_new_term_ref();
	double val = 0.0;

	if (PL_get_dict_key(min_key, interval_term, time_range)) {
		if (PL_get_dict_key(min_key, time_range, inner_arg) &&
			PL_term_type(inner_arg) == PL_FLOAT && PL_get_float(inner_arg, &val)) {
			*minMin = PointType(val);
		}
		if (PL_get_dict_key(max_key, time_range, inner_arg) &&
			PL_term_type(inner_arg) == PL_FLOAT && PL_get_float(inner_arg, &val)) {
			*minMax = PointType(val);
		}
	}

	if (PL_get_dict_key(max_key, interval_term, time_range)) {
		if (PL_get_dict_key(min_key, time_range, inner_arg) &&
			PL_term_type(inner_arg) == PL_FLOAT && PL_get_float(inner_arg, &val)) {
			*maxMin = PointType(val);
		}
		if (PL_get_dict_key(max_key, time_range, inner_arg) &&
			PL_term_type(inner_arg) == PL_FLOAT && PL_get_float(inner_arg, &val)) {
			*maxMax = PointType(val);
		}
	}
}

bool PrologQuery::putScope(const std::shared_ptr<FormulaQuery> &query, term_t pl_scope) {
	static const auto time_key = PL_new_atom("time");
	static const auto confidence_key = PL_new_atom("confidence");

	if (PL_is_variable(pl_scope)) {
		// reasoner did not specify a solution scope
		return false;
	} else if (PL_is_dict(pl_scope)) {
		// the solution scope was generated as a Prolog dictionary
		auto scope_val = PL_new_term_ref();

		// read "time" key, the value is expected to be a dictionary of the form
		//    { min: {min: <double>, max: <double>}, max: {min: <double>, max: <double>}  }
		//    all keys are optional
		if (PL_get_dict_key(time_key, pl_scope, scope_val)) {
			std::optional<TimePoint> minMin, minMax, maxMin, maxMax;
			readFuzzyInterval<TimePoint>(scope_val, &minMin, &minMax, &maxMin, &maxMax);

			if (minMin.has_value() || minMax.has_value() || maxMin.has_value() || maxMax.has_value()) {
				KB_WARN("todo: implement set time interval");
				//query->setTimeInterval(std::make_shared<TimeInterval>(
				//        Range<TimePoint>(minMin, minMax),
				//        Range<TimePoint>(maxMin, maxMax)));
			}
		}

		// read "confidence" key, the value is expected to be a dictionary of the form
		//        { min: {min: <double>, max: <double>}, max: {min: <double>, max: <double>}  }
		//        all keys are optional
		if (PL_get_dict_key(confidence_key, pl_scope, scope_val)) {
			std::optional<ConfidenceValue> minMin, minMax, maxMin, maxMax;
			readFuzzyInterval<ConfidenceValue>(scope_val, &minMin, &minMax, &maxMin, &maxMax);

			if (minMin.has_value() || minMax.has_value() || maxMin.has_value() || maxMax.has_value()) {
				KB_WARN("todo: implement set confidence interval");
				//query->setConfidenceInterval(std::make_shared<ConfidenceInterval>(
				//        Range<ConfidenceValue>(minMin, minMax),
				//        Range<ConfidenceValue>(maxMin, maxMax)));
			}
		}

		PL_reset_term_refs(scope_val);
		return true;
	} else {
		KB_WARN("scope has an unexpected type (should be dict).");
		return false;
	}
}

std::shared_ptr<GraphSelector> PrologQuery::createSolutionFrame(term_t pl_scope) {
	static const auto time_key = PL_new_atom("time");
	static const auto uncertain_key = PL_new_atom("uncertain");
	static const auto occasional_key = PL_new_atom("occasional");
	static const auto confidence_key = PL_new_atom("confidence");
	static const auto agent_key = PL_new_atom("agent");
	static const auto true_a = PL_new_atom("true");

	std::shared_ptr<GraphSelector> frame;

	if (PL_is_variable(pl_scope)) {
		// reasoner did not specify a solution scope
		return {};
	} else if (PL_is_dict(pl_scope)) {
		// the solution scope was generated as a Prolog dictionary
		auto scope_val = PL_new_term_ref();

		frame = std::make_shared<GraphSelector>();

		// read "time" key, the value is expected to be a predicate `range(Since,Until)`
		if (PL_get_dict_key(time_key, pl_scope, scope_val)) {
			term_t arg = PL_new_term_ref();
			std::optional<TimePoint> v_since, v_until;
			double val = 0.0;

			if (PL_get_arg(1, scope_val, arg) &&
				PL_term_type(arg) == PL_FLOAT &&
				PL_get_float(arg, &val) &&
				val > 0.001) { frame->begin = val; }

			if (PL_get_arg(2, scope_val, arg) &&
				PL_term_type(arg) == PL_FLOAT &&
				PL_get_float(arg, &val)) { frame->end = val; }
		}

		if (PL_get_dict_key(uncertain_key, pl_scope, scope_val)) {
			atom_t flagAtom;
			if (PL_term_type(scope_val) == PL_ATOM && PL_get_atom(scope_val, &flagAtom)) {
				if (flagAtom == true_a) {
					frame->epistemicOperator = EpistemicOperator::BELIEF;
				}
			}
		}

		if (PL_get_dict_key(confidence_key, pl_scope, scope_val)) {
			double confidenceValue = 1.0;
			if (PL_term_type(scope_val) == PL_FLOAT && PL_get_float(scope_val, &confidenceValue)) {
				frame->confidence = confidenceValue;
				if (confidenceValue > 0.999) {
					frame->epistemicOperator = EpistemicOperator::KNOWLEDGE;
				} else {
					frame->epistemicOperator = EpistemicOperator::BELIEF;
				}
			}
		}

		if (PL_get_dict_key(occasional_key, pl_scope, scope_val)) {
			atom_t flagAtom;
			if (PL_term_type(scope_val) == PL_ATOM && PL_get_atom(scope_val, &flagAtom)) {
				if (flagAtom == true_a) {
					frame->temporalOperator = TemporalOperator::SOMETIMES;
				} else {
					frame->temporalOperator = TemporalOperator::ALWAYS;
				}
			}
		}

		if (PL_get_dict_key(agent_key, pl_scope, scope_val)) {
			atom_t agentAtom;
			if (PL_term_type(scope_val) == PL_ATOM && PL_get_atom(scope_val, &agentAtom)) {
				if(!frame->epistemicOperator.has_value()) {
					frame->epistemicOperator = EpistemicOperator::KNOWLEDGE;
				}
				frame->agent = Agent::get(PL_atom_chars(agentAtom));
			}
		}

		PL_reset_term_refs(scope_val);
	} else {
		KB_WARN("solution scope has an unexpected type (should be dict).");
	}

	if (frame) {
		return frame;
	} else {
		return {};
	}
}

bool PrologQuery::putScope(term_t pl_term, const AnswerPtr &solution) {
	static const auto time_key = PL_new_atom("time");
	static const auto confidence_key = PL_new_atom("confidenceInterval");

	//auto &timeInterval = solution->timeInterval();
	std::optional<double> confidenceValue = std::nullopt;
	//auto &confidenceValue = solution->modalFrame().confidence();

	int numScopeKeys = 0;
	//if(timeInterval.has_value())    numScopeKeys += 1;
	if (confidenceValue.has_value()) numScopeKeys += 1;
	atom_t scopeKeys[numScopeKeys];
	auto scopeValues = PL_new_term_refs(numScopeKeys);

	if (numScopeKeys > 0) {
		int keyIndex = 0;
		// TODO: re-enable time interval parameter!
		//if(timeInterval.has_value() &&
		//   PrologQuery::putTerm(scopeValues, timeInterval.value())) {
		//    scopeKeys[keyIndex++] = time_key;
		//}
		if (confidenceValue.has_value() &&
			PL_put_float(scopeValues + keyIndex, confidenceValue.value())) {
			scopeKeys[keyIndex++] = confidence_key;
		}
		if (!PL_put_dict(pl_term, 0, numScopeKeys, scopeKeys, scopeValues)) {
			return false;
		}
	}

	return true;
}

/******************************************/
/*********** static constants *************/
/******************************************/

const atom_t &PrologQuery::ATOM_fail() {
	static atom_t a = PL_new_atom("fail");
	return a;
}

const atom_t &PrologQuery::ATOM_false() {
	static atom_t a = PL_new_atom("false");
	return a;
}

const atom_t &PrologQuery::ATOM_true() {
	static atom_t a = PL_new_atom("true");
	return a;
}

const atom_t &PrologQuery::ATOM_comma() {
	static atom_t a = PL_new_atom(",");
	return a;
}

const atom_t &PrologQuery::ATOM_semicolon() {
	static atom_t a = PL_new_atom(";");
	return a;
}

const functor_t &PrologQuery::FUNCTOR_comma() {
	static atom_t a = PL_new_functor(PrologQuery::ATOM_comma(), 2);
	return a;
}

const functor_t &PrologQuery::FUNCTOR_semicolon() {
	static atom_t a = PL_new_functor(PrologQuery::ATOM_semicolon(), 2);
	return a;
}

const predicate_t &PrologQuery::PREDICATE_comma() {
	static predicate_t a = PL_pred(PrologQuery::FUNCTOR_comma(), nullptr);
	return a;
}

const predicate_t &PrologQuery::PREDICATE_semicolon() {
	static predicate_t a = PL_pred(PrologQuery::FUNCTOR_semicolon(), nullptr);
	return a;
}

