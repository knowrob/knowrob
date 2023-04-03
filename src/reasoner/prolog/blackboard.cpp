
#include <SWI-Prolog.h>
#include <filesystem>

#include "knowrob/queries/AnswerStream.h"
#include "knowrob/queries/AnswerQueue.h"
#include "knowrob/reasoner/prolog/PrologQuery.h"
#include "knowrob/reasoner/Blackboard.h"
#include "knowrob/Logger.h"
#include "knowrob/terms/OptionList.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/terms/ListTerm.h"

using namespace knowrob;

/**
 * Context object for qa_call predicate.
 * Evaluates a query in QA system.
 */
class QACallContext {
public:
    std::shared_ptr<Blackboard> blackboard_;
    std::shared_ptr<AnswerQueue> results_;
    std::shared_ptr<Query> query_;
    std::map<std::string, term_t> queryVars_;

    QACallContext(term_t t_reasonerManagerID, term_t t_goal, term_t t_queryCtx)
    : results_(std::make_shared<AnswerQueue>())
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

    void applySolution(const AnswerPtr &solution) {
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

    AnswerPtr nextSolution() {
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
    if(AnswerStream::isEOS(solution)) {
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
PREDICATE(kb_project1, 1)
// +Goal, +Scope, +Options
PREDICATE(kb_unproject, 3)
*/

inline foreign_t qa_remember(ReasonerManager *reasonerManager,
                             const std::filesystem::path &path,
                             const std::string &reasonerName)
{
    auto definedReasoner = reasonerManager->getReasonerWithID(reasonerName);
    if(definedReasoner) {
        if(definedReasoner->reasoner()->hasCapability(CAPABILITY_IMPORT_EXPORT)) {
            KB_INFO("Importing from {} into reasoner '{}'.", path, reasonerName);
            return definedReasoner->reasoner()->importData(path);
        }
        else {
            return true;
        }
    }
    else {
        KB_WARN("reasoner '{}' is not defined.", reasonerName);
        return false;
    }
}

inline foreign_t qa_remember(ReasonerManager *reasonerManager,
                             const std::filesystem::path &path,
                             const OptionList &options)
{
    auto reasonerOpt = options.get("reasoner", Bottom::get());
    // TODO: consider handling include/exlcude option
    //auto includeOpt = options.get("include", ListTerm::nil());
    //auto excludeOpt = options.get("exclude", ListTerm::nil());

    if(reasonerOpt.get() != Bottom::get().get()) {
        // import into one reasoner backend.
        // assume the provided directory contains the exported data.
        const auto &reasonerName = ((StringTerm*)reasonerOpt.get())->value();
        return qa_remember(reasonerManager, path, reasonerName);
    }
    else {
        for(auto &subdir : std::filesystem::directory_iterator(path)) {
            // assume subdirectories are named after reasoner
            if(is_directory(subdir)) {
                // get reasoner from reasoner name
                const auto reasonerName = subdir.path().filename();
                qa_remember(reasonerManager, subdir, reasonerName);
            }
        }
        return true;
    }
}

foreign_t pl_qa_remember3(term_t t_reasonerManager, term_t t_path, term_t t_options)
{
    int reasonerManagerID=0;
    char *path_str;
    if(PL_get_integer(t_reasonerManager, &reasonerManagerID) &&
       PL_get_atom_chars(t_path, &path_str))
    {
        auto reasonerManager = ReasonerManager::getReasonerManager(reasonerManagerID);
        if(reasonerManager) {
            auto optionTerm = PrologQuery::constructTerm(t_options);
            return qa_remember(reasonerManager, path_str, OptionList(optionTerm));
        }
    }
    return false;
}

inline foreign_t qa_memorize(const std::shared_ptr<IReasoner> &reasoner,
                             const std::string &reasonerName,
                             const std::filesystem::path &path)
{
    if(reasoner->hasCapability(CAPABILITY_IMPORT_EXPORT)) {
        KB_INFO("Exporting from {} from reasoner '{}'.", path, reasonerName);
        return reasoner->exportData(path);
    }
    else {
        return true;
    }
}

inline foreign_t qa_memorize(ReasonerManager *reasonerManager,
                             const std::filesystem::path &path,
                             const OptionList &options)
{
    auto reasonerOpt = options.get("reasoner", Bottom::get());
    // TODO: consider handling include/exlcude option
    //auto includeOpt = options.get("include", ListTerm::nil());
    //auto excludeOpt = options.get("exclude", ListTerm::nil());

    if(!exists(path)) create_directories(path);

    if(reasonerOpt.get() != Bottom::get().get()) {
        // export from one reasoner backend.
        const auto &reasonerName = ((StringTerm*)reasonerOpt.get())->value();
        auto definedReasoner = reasonerManager->getReasonerWithID(reasonerName);
        if(definedReasoner) {
            return qa_memorize(definedReasoner->reasoner(), reasonerName, path);
        }
        else {
            KB_WARN("reasoner '{}' is not defined.", reasonerName);
            return false;
        }
    }
    else {
        for(auto &pair : reasonerManager->reasonerPool()) {
            auto subdir = path / pair.first;
            if(!exists(subdir)) create_directories(subdir);
            qa_memorize(pair.second->reasoner(), pair.first, subdir);
        }
        return true;
    }
}

foreign_t pl_qa_memorize3(term_t t_reasonerManager, term_t t_path, term_t t_options)
{
    int reasonerManagerID=0;
    char *path_str;
    if(PL_get_integer(t_reasonerManager, &reasonerManagerID) &&
       PL_get_atom_chars(t_path, &path_str))
    {
        auto reasonerManager = ReasonerManager::getReasonerManager(reasonerManagerID);
        if(reasonerManager) {
            auto optionTerm = PrologQuery::constructTerm(t_options);
            return qa_memorize(reasonerManager, path_str, OptionList(optionTerm));
        }
    }
    return false;
}

PL_extension qa_predicates[] = {
        { "qa_call", 2, (pl_function_t)pl_qa_call2, PL_FA_NONDETERMINISTIC },
        { "qa_call", 4, (pl_function_t)pl_qa_call4, PL_FA_NONDETERMINISTIC },
        { "qa_remember", 3, (pl_function_t)pl_qa_remember3, 0 },
        { "qa_memorize", 3, (pl_function_t)pl_qa_memorize3, 0 },
        { nullptr,   0, nullptr, 0 }
};
