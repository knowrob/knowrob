/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"
#include "knowrob/reasoner/prolog/PrologQueryRunner.h"

using namespace knowrob;

PrologQueryRunner::PrologQueryRunner(PrologReasoner *reasoner, Request request, bool sendEOS)
: ThreadPool::Runner(),
  reasoner_(reasoner),
  request_(std::move(request)),
  sendEOS_(sendEOS)
{
}

void PrologQueryRunner::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;

	KB_DEBUG("PrologReasoner has new query {}:({}).",
			 request_.queryModule, *request_.goal);
	// use the reasoner module as context module for query evaluation
	module_t ctx_module = PL_new_module(PL_new_atom(request_.queryModule->value().c_str()));
	// the exception risen by the Prolog engine, if any
	auto pl_exception = (term_t)0;

	// construct two arguments: `b_setval(reasoner_module, $request_.queryModule)` and `$request_.goal`
	PrologQuery pl_goal(request_.goal);
	auto solution_scope  = PL_new_term_ref();
	auto instantiations  = PL_new_term_ref();
	auto query_args = createQueryArgumentTerms(pl_goal, solution_scope, instantiations);
	if(query_args==(term_t)0) {
		KB_WARN("failed to construct Prolog query `{}`.", *request_.goal);
		stop(false);
	}
	// open a Prolog query.
	auto qid = PL_open_query(
			ctx_module,                        // the context module of the goal.
			flags,                                // querying flags
			PrologQuery::PREDICATE_comma(),  // the goal predicate
			query_args);                       // argument vector

	// do the query processing
	while(!hasStopRequest()) {
		// here is where the main work is done
		if(!PL_next_solution(qid)) {
			// read exception, if any
			pl_exception = PL_exception(qid);
			break;
		}
		// handle stop request
		if(hasStopRequest()) break;

		KB_DEBUG("Prolog has a next solution for query `{}`.", request_.queryID);
		// create an empty solution
		auto solution = std::make_shared<QueryResult>();
		// add substitutions
		for(const auto& kv: pl_goal.vars()) {
			solution->substitute(Variable(kv.first), PrologQuery::constructTerm(kv.second));
		}
		// store instantiations of predicates
		if(PL_is_list(instantiations)){
			auto head = PL_new_term_ref(); /* the elements */
			auto list = PL_copy_term_ref(instantiations);
			while(PL_get_list(list, head, list)) {
				auto t = PrologQuery::constructTerm(head);
				solution->addPredicate(reasoner_->reasonerIDTerm_,
									   std::static_pointer_cast<Predicate>(t));
			}
		}
		// set the solution scope, if reasoner specified it
		setSolutionScope(solution, solution_scope);
		// push the solution into the output stream
		request_.queryInstance->pushSolution(solution);
	}

	// construct exception term
	TermPtr exceptionTerm;
	if(pl_exception != (term_t)0) {
		exceptionTerm = PrologQuery::constructTerm(pl_exception);
		PL_clear_exception();
		KB_DEBUG("Prolog query failed.");
	}
	else {
		KB_DEBUG("Prolog query completed.");
	}

	// free up resources
	PL_close_query(qid);
	// notify PrologReasoner about runner being done
	reasoner_->finishRunner(request_.queryID, this);

	// make sure EOS is published on output stream
	if(sendEOS_) request_.queryInstance->pushEOS();
	// throw error if Prolog evaluation has caused an exception.
	if(exceptionTerm) throw QueryError(*request_.goal, *exceptionTerm);
}

void PrologQueryRunner::setSolutionScope(std::shared_ptr<QueryResult> &solution, term_t solutionScope)
{
    static const auto time_key = PL_new_atom("time");
    static const auto confidence_key = PL_new_atom("confidence");

    if(PL_is_variable(solutionScope)) {
        // reasoner did not specify a solution scope
        return;
    }
    else if(PL_is_dict(solutionScope)) {
        // the solution scope was generated as a Prolog dictionary
        auto scope_val = PL_new_term_ref();

        // read "time" key, the value is expected to be a predicate `range(Since,Until)`
        if(PL_get_dict_key(time_key, solutionScope, scope_val)) {
            term_t arg = PL_new_term_ref();
            std::optional<TimePoint> v_since, v_until;
            double val=0.0;

            if(PL_get_arg(1, scope_val, arg) &&
               PL_term_type(arg)==PL_FLOAT &&
               PL_get_float(arg, &val) &&
               val > 0.001)
            { v_since = val; }

            if(PL_get_arg(2, scope_val, arg) &&
               PL_term_type(arg)==PL_FLOAT &&
               PL_get_float(arg, &val))
            { v_until = val; }

            if(v_since.has_value() || v_until.has_value()) {
                solution->setTimeInterval(std::make_shared<TimeInterval>(
                        Range<TimePoint>(v_since, std::nullopt),
                        Range<TimePoint>(std::nullopt, v_until)));
            }
        }

        // read "confidence" key, the value is expected to be a float value
        if(PL_get_dict_key(confidence_key, solutionScope, scope_val)) {
            double v_confidence=0.0;
            if(PL_term_type(scope_val)==PL_FLOAT &&
               PL_get_float(scope_val, &v_confidence))
            {
                solution->setConfidenceValue(
                        std::make_shared<ConfidenceValue>(v_confidence));
            }
        }

        PL_reset_term_refs(scope_val);
    }
    else {
        KB_WARN("solution scope has an unexpected type (should be dict).");
    }
}

term_t PrologQueryRunner::createContextTerm(
        term_t solutionScopeVar, term_t predicatesVar)
{
    static const auto time_key = PL_new_atom("time");
    static const auto confidence_key = PL_new_atom("confidenceInterval");

    static const auto query_scope_f = PL_new_functor(PL_new_atom("query_scope"), 1);
    static const auto solution_scope_f = PL_new_functor(PL_new_atom("solution_scope"), 1);
    static const auto predicates_f = PL_new_functor(PL_new_atom("predicates"), 1);

    auto &timeInterval = request_.queryInstance->timeInterval();
    auto &confidenceInterval = request_.queryInstance->confidenceInterval();

    int numScopeKeys = 0;
    if(timeInterval.has_value())       numScopeKeys += 1;
    if(confidenceInterval.has_value()) numScopeKeys += 1;

    // create an option list
    auto listTerm = PL_new_term_ref();
    if(!PL_put_nil(listTerm)) return (term_t)0;

    // option: query_scope($dictTerm)
    if(numScopeKeys>0) {
        atom_t scopeKeys[numScopeKeys];
        auto scopeValues = PL_new_term_refs(numScopeKeys);

        int keyIndex = 0;
        if(timeInterval.has_value()) {
            createIntervalDict(scopeValues, *timeInterval.value());
            scopeKeys[keyIndex++] = time_key;
        }
        if(confidenceInterval.has_value()) {
            createIntervalDict(scopeValues+keyIndex, *confidenceInterval.value());
            scopeKeys[keyIndex++] = confidence_key;
        }

        auto dictTerm = PL_new_term_ref();
        auto queryScopeOption = PL_new_term_ref();
        if(!PL_put_dict(dictTerm, 0, numScopeKeys, scopeKeys, scopeValues) ||
           !PL_cons_functor(queryScopeOption, query_scope_f, dictTerm) ||
           !PL_cons_list(listTerm, queryScopeOption, listTerm))
        {
            return (term_t)0;
        }
    }

    // option: solution_scope(solutionScopeVar)
    auto solutionScopeOption = PL_new_term_ref();
    if(!PL_cons_functor(solutionScopeOption, solution_scope_f, solutionScopeVar) ||
       !PL_cons_list(listTerm, solutionScopeOption, listTerm))
    {
        return (term_t)0;
    }

    // option: predicates(predicatesVar)
    auto predicatesOption = PL_new_term_ref();
    if(!PL_cons_functor(predicatesOption, predicates_f, predicatesVar) ||
       !PL_cons_list(listTerm, predicatesOption, listTerm))
    {
        return (term_t)0;
    }

    return listTerm;
}

term_t PrologQueryRunner::createQueryArgumentTerms(
        PrologQuery &pl_goal, term_t solutionScopeVar, term_t predicatesVar)
{
    static const auto reasoner_module_a = PL_new_atom("reasoner_module");
    static const auto b_setval_f = PL_new_functor(PL_new_atom("b_setval"), 2);
    const auto call_f = request_.callFunctor;

    // construct b_setval/2 arguments
    auto setval_args = PL_new_term_refs(2);
    if(!PL_put_atom(setval_args, reasoner_module_a) ||
       !PL_put_atom_chars(setval_args+1, request_.queryModule->value().c_str()))
    {
        return (term_t)0;
    }

    // construct arguments for the comma predicate
    auto query_args  = PL_new_term_refs(2);
    // first argument is the predicate b_setval/2
    if(!PL_cons_functor_v(query_args, b_setval_f, setval_args)) return (term_t)0;
    // second argument is the query, optionally wrapped in a call/2 predicate
    if(call_f != (functor_t)0) {
        // construct arguments for call/2 predicate
        auto call_args = PL_new_term_refs(2);
        if(!PL_put_term(call_args, pl_goal.pl_query()) ||
           !PL_put_term(call_args+1, createContextTerm(solutionScopeVar,predicatesVar)) ||
           !PL_cons_functor_v(query_args+1, call_f, call_args))
        {
            return (term_t)0;
        }
    }
    else {
        if(!PL_put_term(query_args+1, pl_goal.pl_query())) return (term_t)0;
    }

    return query_args;
}

template <class T>
bool PrologQueryRunner::createRangeDict(term_t intervalDict, const Range<T> &range)
{
    static const auto min_key = PL_new_atom("min");
    static const auto max_key = PL_new_atom("max");

    int numRangeKeys = 0;
    if(range.min().has_value()) numRangeKeys += 1;
    if(range.max().has_value()) numRangeKeys += 1;
    atom_t rangeKeys[numRangeKeys];
    auto rangeValues = PL_new_term_refs(numRangeKeys);

    int keyIndex = 0;
    if(range.min().has_value()) {
        if(!PL_put_float(rangeValues, range.min().value().value())) return false;
        rangeKeys[keyIndex++] = min_key;
    }
    if(range.max().has_value()) {
        if(!PL_put_float(rangeValues+keyIndex, range.max().value().value())) return false;
        rangeKeys[keyIndex++] = max_key;
    }

    return PL_put_dict(intervalDict, 0, numRangeKeys, rangeKeys, rangeValues);
}

template <class T>
bool PrologQueryRunner::createIntervalDict(term_t intervalDict, const FuzzyInterval<T> &i)
{
    static const auto min_key = PL_new_atom("min");
    static const auto max_key = PL_new_atom("max");

    int numRangeKeys = 0;
    if(i.minRange().hasValue()) numRangeKeys += 1;
    if(i.maxRange().hasValue()) numRangeKeys += 1;
    atom_t rangeKeys[numRangeKeys];
    auto rangeValues = PL_new_term_refs(numRangeKeys);

    int keyIndex = 0;
    if(i.minRange().hasValue()) {
        createRangeDict(rangeValues, i.minRange());
        rangeKeys[keyIndex++] = min_key;
    }
    if(i.maxRange().hasValue()) {
        createRangeDict(rangeValues+keyIndex, i.maxRange());
        rangeKeys[keyIndex++] = max_key;
    }

    return PL_put_dict(intervalDict, 0,numRangeKeys, rangeKeys, rangeValues);
}
