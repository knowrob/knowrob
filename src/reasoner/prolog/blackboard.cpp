
#include <SWI-Prolog.h>

#include "knowrob/queries/QueryResultStream.h"
#include "knowrob/queries/QueryResultQueue.h"
#include "knowrob/reasoner/prolog/PrologQuery.h"
#include "knowrob/Blackboard.h"
#include "knowrob/Logger.h"

using namespace knowrob;

/**
 * Context object for qa_call predicate.
 * Evaluates a query in QA system.
 */
class QACallContext {
public:
    std::shared_ptr<Blackboard> blackboard_;
    std::shared_ptr<QueryResultQueue> results_;
    std::shared_ptr<Query> query_;
    std::map<std::string, term_t> queryVars_;

    QACallContext(term_t t_reasonerManagerID, term_t t_goal, term_t t_queryCtx)
    : results_(std::make_shared<QueryResultQueue>())
    {
        // create query object
        query_ = PrologQuery::toQuery(PrologQuery::constructTerm(t_goal, &queryVars_));
        // set query context from context term t_queryCtx
        if(t_queryCtx != (term_t)0) {
            PrologQuery::putScope(query_, t_queryCtx);
        }
        // find the reasoner manager
        int reasonerManagerID=0;
        if(!PL_get_integer(t_reasonerManagerID, &reasonerManagerID)) reasonerManagerID=0;
        auto reasonerManager = ReasonerManager::getReasonerManager(reasonerManagerID);
        // create blackboard for the query
        blackboard_ = std::make_shared<Blackboard>(reasonerManager, results_, query_);
        blackboard_->start();
    }

    void applySolution(const QueryResultPtr &solution) {
        // FIXME: for some reason this does not work as expected.
        //    idea is to remember all variables on PL_FIRST_CALL to quickly instantiate them
        //    with each solution. for now PL_unify is used instead.
        for(auto &pair : *solution->substitution()) {
            auto pl_term_it = queryVars_.find(pair.first.name());
            if(pl_term_it != queryVars_.end()) {
                PrologQuery::putTerm(pl_term_it->second, pair.second, queryVars_);
            } else {
                KB_WARN("unknown variable {}", pair.first.name());
            }
        }
    }

    QueryResultPtr nextSolution() {
        return results_->pop_front();
    }

    bool hasMoreSolutions() {
        return !results_->empty() || results_->isOpened();
    }
};

foreign_t pl_qa_call4(term_t t_reasonerManager, term_t t_goal,
                      term_t t_queryCtx, term_t t_resultCtx,
                      control_t handle)
{
    // get, create, or delete call context
    QACallContext *callContext;
    switch(PL_foreign_control(handle)) {
        case PL_FIRST_CALL:
            // create a new call context
            callContext = new QACallContext(t_reasonerManager, t_goal, t_queryCtx);
            break;
        case PL_REDO:
            // pop next choice point
            callContext = (QACallContext*)PL_foreign_context_address(handle);
            break;
        case PL_PRUNED:
            // remaining choice points were pruned
            callContext = (QACallContext*)PL_foreign_context_address(handle);
            delete callContext;
            return TRUE;
    }
    // callContext is an existing context below, try to get next solution
    // TODO: list of instantiated predicates is lost here, need to hand it back to Prolog!
    auto solution = callContext->nextSolution();
    if(QueryResultStream::isEOS(solution)) {
        // no solution, fail
        delete callContext;
        return FALSE;
    }
    // apply substitution mapping
    auto pl_queryInstance = PL_new_term_ref();
    auto queryInstance = callContext->query_->applySubstitution(*solution->substitution());
    PrologQuery::putTerm(pl_queryInstance, queryInstance->formula(), callContext->queryVars_);
    if(!PL_unify(t_goal, pl_queryInstance)) {
        // something went wrong, fail
        delete callContext;
        return FALSE;
    }

    // put scope term into t_resultCtx
    if(t_resultCtx != (term_t)0) {
        PrologQuery::putScope(t_resultCtx, solution);
    }
    if(!callContext->hasMoreSolutions()) {
        // succeed without a choice point
        delete callContext;
        return TRUE;
    }
    // succeed with a choice point
    PL_retry_address(callContext);
}

foreign_t pl_qa_call2(term_t t_reasonerManager, term_t t_goal, control_t handle)
{
    return pl_qa_call4(t_reasonerManager, t_goal,
                       (term_t)0, (term_t)0, handle);
}

/*
// +Goal, +Options
PREDICATE(kb_project1, 1) {
return TRUE;
}

// +Goal, +Scope, +Options
PREDICATE(kb_unproject, 3) {
return TRUE;
}

PREDICATE(memorize, 3) {

%% memorize(+Directory) is det.
%
% Store knowledge into given directory.
%
% @param Directory filesystem path
%
memorize(Directory) :-
mng_export(Directory).
return TRUE;
}

PREDICATE(remember, 3) {
%% remember(+Directory) is det.
%
% Restore memory previously stored into given directory.
%
% @param Directory filesystem path
%
remember(Directory) :-
mng_import(Directory).
return TRUE;
}
*/

PL_extension qa_predicates[] = {
        { "qa_call", 2, (pl_function_t)pl_qa_call2, PL_FA_NONDETERMINISTIC },
        { "qa_call", 4, (pl_function_t)pl_qa_call4, PL_FA_NONDETERMINISTIC },
        { nullptr,   0, nullptr, 0 }
};
