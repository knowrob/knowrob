/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/reasoning/prolog/PrologReasoner.h>

using namespace knowrob;

// TODO: it seems all engines share the same data.
//           so when one loads a rule, it is available for all.
//           can this be controlled? here I would like that each PrologReasoner can have its
//           own set of data loaded.
//           so the PrologEngine would need to accept a data source parameter.

// TODO: support synchronization with data sources.
//      - when facts are asserted into EDB, also assert into PrologEngine
//      - support writing facts asserted from PrologEngine into EDB?
//           i.e. when assert is called in the PrologEngine.

PrologReasoner::PrologReasoner(boost::shared_ptr<IFactBase> &edb, boost::shared_ptr<IRuleBase> &idb)
: LogicProgramReasoner(edb,idb)
{
}

PrologReasoner::~PrologReasoner()
{
    // TODO: need to block until run finished?
}

void PrologReasoner::initialize()
{
    // claim a prolog engine where facts and rules can be asseted
    boost::shared_ptr<PrologEngine> engine = prologEnginePool_.claim();
    // load facts from EDB
    Iterator<Predicate> factIt = edb_.getFacts();
    while(factIt.hasNext()) {
        engine->assert(factIt.next());
    }
    // load rules from IDB
    Iterator<Rule> ruleIt = idb_.getRules();
    while(ruleIt.hasNext()) {
        engine->assert(ruleIt.next());
    }
    // release the engine again
    prologEnginePool_.release(engine);
}

bool PrologReasoner::canReasonAbout(const PredicateIndicator &predicate)
{
    // TODO: better check if PrologEngine knows the predicate?
    return edb_.containsPredicate() || idb_.containsPredicate();
}

void PrologReasoner::run(const IQuery &goal, ReasoningStatus &status, MessageQueue<Answer> &answerQueue)
{
    // claim a prolog engine
    boost::shared_ptr<PrologEngine> engine = prologEnginePool_.claim();
    // run the query
	engine->startQuery(goal, true);
	while(!status.isCancelled() && engine->hasMoreSolutions()) {
        answerQueue.push(engine->popSolution());
	}
	engine->stopQuery(true);
    // release the engine again
    prologEnginePool_.release(engine);
}
