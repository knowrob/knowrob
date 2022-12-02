/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/reasoning/prolog/PrologReasoner.h>
#include <knowrob/reasoning/prolog/PrologEngine.h>

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

PrologReasoner::PrologReasoner(std::shared_ptr<PrologPool> &prologEnginePool, const std::string &initFile)
: prologEnginePool_(prologEnginePool),
  initFile_(initFile)
{
}

PrologReasoner::~PrologReasoner()
{
	// TODO: need to block until run finished?
}

void PrologReasoner::initialize()
{
	// claim a prolog engine where facts and rules can be asseted
	std::shared_ptr<PrologEngine> engine = prologEnginePool_->claim();
	// consult the init file, i.e. load facts and rules declared
	// in the file, and execute directives it contains
	engine->consult(initFile_);
	// TODO: load any additional rules stored in IDBs
	//for(auto it=idbs_.begin(); it!=idbs_.end(); it++) {
	//}
	if(!idbs_.empty()) {
		// TODO: print warning "PrologEngine does not support the use of additional IDBs as of now."
	}
	// load any additional facts stored in EDBs
	for(auto it=edbs_.begin(); it!=edbs_.end(); it++) {
		std::shared_ptr<IFactBase> edb = *it;
		for(const std::shared_ptr<Predicate> &p : *edb) {
			engine->assertFact(p);
		}
	}
	// release the engine again
	prologEnginePool_->release(engine);
}

void PrologReasoner::consult(const std::string &prologFile)
{
	std::shared_ptr<PrologEngine> engine = prologEnginePool_->claim();
	engine->consult(prologFile);
	prologEnginePool_->release(engine);
}

void PrologReasoner::assertFact(const std::shared_ptr<Predicate> &fact)
{
	std::shared_ptr<PrologEngine> engine = prologEnginePool_->claim();
	engine->assertFact(fact);
	prologEnginePool_->release(engine);
}

void PrologReasoner::runQuery(
		std::shared_ptr<Query> &goal,
		ReasoningStatus &status,
		std::shared_ptr<QueryResultQueue> &answerQueue)
{
	// claim a prolog engine
	std::shared_ptr<PrologEngine> engine = prologEnginePool_->claim();
	// run the query
	// TODO: might be prolog supports a better way to terminate a running
	//       inference. Here it can only be terminated in between two answers
	//       that are generated.
	engine->startQuery(goal, true);
	while(!status.isCancelled() && engine->hasMoreSolutions()) {
		answerQueue->push(engine->nextSolution());
	}
	// TODO: need to block here? (flag=true means to block)
	engine->stopQuery(true);
	// release the engine again
	prologEnginePool_->release(engine);
}

bool PrologReasoner::canReasonAbout(const PredicateIndicator &predicate)
{
	std::shared_ptr<PrologEngine> engine = prologEnginePool_->claim();
	// run query "current_functor(p,n)"
	bool hasSolution = engine->oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate("current_functor",
			std::vector<std::shared_ptr<Term>>{
				std::shared_ptr<Term>(new StringAtom(predicate.functor())),
				std::shared_ptr<Term>(new Integer32Atom(predicate.arity()))
			}
		))
	)))->hasSolution();
	prologEnginePool_->release(engine);
	return hasSolution;
}

