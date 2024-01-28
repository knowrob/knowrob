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
#include "knowrob/KnowledgeBase.h"

using namespace knowrob;

PrologQueryRunner::PrologQueryRunner(PrologReasoner *reasoner,
                Request request,
                const std::shared_ptr<AnswerStream::Channel> &outputChannel,
                bool sendEOS)
: ThreadPool::Runner(),
  reasoner_(reasoner),
  request_(std::move(request)),
  sendEOS_(sendEOS),
  outputChannel_(outputChannel)
{
}

void PrologQueryRunner::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;

	KB_DEBUG("PrologReasoner has new query {}:({}).",
			 request_.queryModule->value(), *request_.goal);
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
		auto solution = std::make_shared<Answer>();
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
        PrologQuery::putScope(solution, solution_scope);
		// push the solution into the output stream
		outputChannel_->push(solution);

		if(request_.goal->flags() & QUERY_FLAG_ONE_SOLUTION) break;
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
	//reasoner_->finishRunner(request_.queryID, this);

	// make sure EOS is published on output stream
	if(sendEOS_) {
        outputChannel_->push(AnswerStream::eos());
    }
	// throw error if Prolog evaluation has caused an exception.
	if(exceptionTerm) throw QueryError(
	    "An exception occurred during the evaluation of literal {}: {}.",
	        *request_.goal, *exceptionTerm);
}

term_t PrologQueryRunner::createContextTerm(
        term_t solutionScopeVar, term_t predicatesVar)
{
    // query frame has the form:
    // { epistemicMode: knowledge|belief,
    //   temporalMode: sometimes|always,
    //   [agent: $name,]
    //   [since: $time,]
    //   [until: $time] }

    static const auto epistemicMode_a = PL_new_atom("epistemicMode");
    static const auto temporalMode_a = PL_new_atom("temporalMode");
    static const auto agent_a = PL_new_atom("agent");
    static const auto knowledge_a = PL_new_atom("knowledge");
    static const auto belief_a = PL_new_atom("belief");
    static const auto sometimes_a = PL_new_atom("sometimes");
    static const auto always_a = PL_new_atom("always");
    static const auto since_a = PL_new_atom("since");
    static const auto until_a = PL_new_atom("until");

    static const auto query_scope_f = PL_new_functor(PL_new_atom("query_scope"), 1);
    static const auto solution_scope_f = PL_new_functor(PL_new_atom("solution_scope"), 1);
    static const auto predicates_f = PL_new_functor(PL_new_atom("predicates"), 1);

    auto &selector = request_.graphSelector;
    int numFrameKeys = 2;
    if(selector.agent.has_value()) numFrameKeys +=1;
    if(selector.begin.has_value()) numFrameKeys += 1;
    if(selector.end.has_value()) numFrameKeys += 1;

    // create an option list
    auto listTerm = PL_new_term_ref();
    if(!PL_put_nil(listTerm)) return (term_t)0;

    // option: frame($dictTerm)
    atom_t scopeKeys[numFrameKeys];
    auto scopeValues = PL_new_term_refs(numFrameKeys);

    // epistemicMode: knowledge|belief
    bool isAboutBelief = (selector.epistemicOperator &&
    	selector.epistemicOperator.value() == EpistemicOperator::BELIEF);
    int keyIndex = 0;
    scopeKeys[keyIndex] = epistemicMode_a;
    if(!PL_put_atom(scopeValues, isAboutBelief ? belief_a : knowledge_a)) return (term_t)0;

    // temporalMode: sometimes|always
    bool isAboutSomePast = (selector.temporalOperator &&
    	selector.temporalOperator.value() == TemporalOperator::SOMETIMES);
    scopeKeys[++keyIndex] = temporalMode_a;
    if(!PL_put_atom(scopeValues+keyIndex, isAboutSomePast ? sometimes_a : always_a)) return (term_t)0;

    // agent: $name
    if(selector.agent.has_value()) {
        scopeKeys[++keyIndex] = agent_a;
        auto &agent_iri = selector.agent.value()->iri();
        if(!PL_put_atom_chars(scopeValues+keyIndex, agent_iri.c_str())) return (term_t)0;
    }

	// since: $name
	if(selector.begin.has_value()) {
		scopeKeys[++keyIndex] = since_a;
		if(!PL_put_float(scopeValues+keyIndex, selector.begin.value())) return (term_t)0;
	}
	// until: $name
	if(selector.end.has_value()) {
		scopeKeys[++keyIndex] = until_a;
		if(!PL_put_float(scopeValues+keyIndex, selector.end.value())) return (term_t)0;
	}

    auto dictTerm = PL_new_term_ref();
    auto frameOption = PL_new_term_ref();
    if(!PL_put_dict(dictTerm, 0, numFrameKeys, scopeKeys, scopeValues) ||
       !PL_cons_functor(frameOption, query_scope_f, dictTerm) ||
       !PL_cons_list(listTerm, frameOption, listTerm))
    {
        return (term_t)0;
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
    static const auto reasoner_manager_a = PL_new_atom("reasoner_manager");
    static const auto b_setval_f = PL_new_functor(PL_new_atom("b_setval"), 2);
    const auto call_f = request_.callFunctor;

    // construct b_setval/2 arguments
    auto setval_args1 = PL_new_term_refs(2);
    if(!PL_put_atom(setval_args1, reasoner_module_a) ||
       !PL_put_atom_chars(setval_args1+1, request_.queryModule->value().c_str()))
    {
        return (term_t)0;
    }
    auto setval_args2 = PL_new_term_refs(2);
    if(!PL_put_atom(setval_args2, reasoner_manager_a) ||
       !PL_put_integer(setval_args2+1, reasoner_->reasonerManagerID()))
    {
        return (term_t)0;
    }
    auto setval_args3 = PL_new_term_refs(2);
    if(!PL_cons_functor_v(setval_args3, b_setval_f, setval_args1) ||
       !PL_cons_functor_v(setval_args3+1, b_setval_f, setval_args2)) {
        return (term_t)0;
    }

    // construct arguments for the comma predicate
    auto query_args  = PL_new_term_refs(2);
    // first argument is the predicate b_setval/2
    if(!PL_cons_functor_v(query_args, PrologQuery::FUNCTOR_comma(), setval_args3)) return (term_t)0;
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
