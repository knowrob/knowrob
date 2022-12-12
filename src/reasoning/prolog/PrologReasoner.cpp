/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// logging
#include <spdlog/spdlog.h>
// KnowRob
#include <knowrob/reasoning/prolog/PrologReasoner.h>

using namespace knowrob;

// TODO: support synchronization with data sources.
//      - when facts are asserted into EDB, also assert into PrologEngine
//      - support writing facts asserted from PrologEngine into EDB?
//           i.e. when assert is called in the PrologEngine.
//      - use Prolog modules to encapsulate reasoner configuriations

PrologReasoner::PrologReasoner(const std::string &initFile)
: threadPool_(std::thread::hardware_concurrency()),
  initFile_(initFile)
{
}

PrologReasoner::~PrologReasoner()
{
	for(auto &pair : activeQueries_) {
		for(auto &x : pair.second.runner) {
			x->stop(true);
		}
	}
	activeQueries_.clear();
}

void PrologReasoner::initialize()
{
	// consult the init file, i.e. load facts and rules declared
	// in the file, and execute directives it contains
	consult(initFile_);
	
	// TODO: load any additional rules stored in IDBs
	//for(auto it=idbs_.begin(); it!=idbs_.end(); it++) {
	//}
	
	// load any additional facts stored in EDBs
	for(auto it=edbs_.begin(); it!=edbs_.end(); it++) {
		std::shared_ptr<IFactBase> edb = *it;
		for(const std::shared_ptr<Predicate> &p : *edb) {
			assertFact(p);
		}
	}
}


bool PrologReasoner::canReasonAbout(const PredicateIndicator &predicate)
{
	return !QueryResultStream::isEOS(oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate(
			"current_functor", std::vector<std::shared_ptr<Term>>{
				std::shared_ptr<Term>(new StringTerm(predicate.functor())),
				std::shared_ptr<Term>(new Integer32Term(predicate.arity()))
			}
		))
	))));
}

bool PrologReasoner::consult(const std::string &prologFile)
{
	return !QueryResultStream::isEOS(oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate(
			"user:consult", std::vector<std::shared_ptr<Term>>{
				std::shared_ptr<Term>(new StringTerm(prologFile))
			}
		))
	))));
}

bool PrologReasoner::assertFact(const std::shared_ptr<Predicate> &fact)
{
	return !QueryResultStream::isEOS(oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate(
			"user:assertz",
			std::vector<std::shared_ptr<Term>>{ fact }
		))
	))));
}


std::shared_ptr<QueryResult> PrologReasoner::oneSolution(const std::shared_ptr<Query> &goal)
{
	std::shared_ptr<QueryResult> result;
	
	// create an output queue for the query
	auto outputStream = std::shared_ptr<QueryResultQueue>(new QueryResultQueue);
	auto outputChannel = outputStream->createChannel();
	// create a runner for a worker thread
	auto workerGoal = std::shared_ptr<PrologReasoner::Runner>(
		new PrologReasoner::Runner(outputChannel, goal));
	
	threadPool_.pushWork(workerGoal);
	return outputStream->pop_front();
}

std::list<std::shared_ptr<QueryResult>> PrologReasoner::allSolutions(const std::shared_ptr<Query> &goal)
{
	std::list<std::shared_ptr<QueryResult>> results;
	std::shared_ptr<QueryResult> nextResult;
	
	// create an output queue for the query
	auto outputStream = std::shared_ptr<QueryResultQueue>(new QueryResultQueue);
	auto outputChannel = outputStream->createChannel();
	// create a runner for a worker thread
	auto workerGoal = std::shared_ptr<PrologReasoner::Runner>(
		new PrologReasoner::Runner(outputChannel, goal));
	
	// assign the goal to a worker thread
	threadPool_.pushWork(workerGoal);
	// get all results
	while(true) {
		nextResult = outputStream->pop_front();
		
		if(QueryResultStream::isEOS(nextResult)) {
			break;
		}
		else {
			results.push_back(nextResult);
		}
	}
	
	return results;
}


void PrologReasoner::startQuery(uint32_t queryID,
	const std::shared_ptr<QueryResultStream::Channel> &outputStream,
	const std::shared_ptr<Query> &goal)
{
	// create a request object and store it in a map
	activeQueries_[queryID] = {
		outputStream,
		goal,
		std::list<std::shared_ptr<PrologReasoner::Runner>>()
	};
	spdlog::debug("query {} started.", queryID);
	spdlog::debug("query {} readable string is {}.", queryID, goal->getHumanReadableString());
}

void PrologReasoner::finishQuery(uint32_t queryID,
	bool isImmediateStopRequested)
{
	// Get query request from query ID
	auto it = activeQueries_.find(queryID);
	if(it == activeQueries_.end()) {
		spdlog::warn("query {} is not active.", queryID);
		return;
	}
	
	if(isImmediateStopRequested) {
		// instruct all active query runners to stop.
		// This basically attempts to gracefully stop them just by toggling
		// on a flag that makes the runner break out its loop in the next iteration,
		// or right away if the thread is sleeping.
		// note: no need to wait for the runner to be stopped here. it could be the runner
		// needs more time to terminate, or even never terminates.
		for(auto &x : it->second.runner) {
			x->stop(false);
		}
	}
	else {
		// wait until every query runner completed to avoid their solutions
		// being published after the EOS message below.
		// note: this may block for a long time if the query evaluation does not terminate,
		// or takes very long time.
		// TODO: It would be good to use a timeout and a max
		// computation time for each runner to avoid deadlocks here.
		// also a mechanism for ungracefully stopping a runner would be good.
		// FIXME: probably not good to block the main thread here!
		for(auto &x : it->second.runner) {
			x->join();
		}
	}
	
	// push EOS message to indicate to subscribers that no more
	// messages will be published on this channel.
	it->second.outputStream->push(QueryResultStream::eos());
	
	// remove this request from the map of active query requests
	activeQueries_.erase(it);
}

void PrologReasoner::pushSubstitution(uint32_t queryID,
	const SubstitutionPtr &bindings)
{
	// TODO: decide how to handle different instantiations of the same query
	//   1. Only do one query + foreign predicate that connects to the stream
	//      this might be something to look into: https://www.swi-prolog.org/pldoc/man?section=foreign-yield
	//      "... simply handle a connection using the loop below which restarts the query as long as it yields."
	//   2. I guess one could fiddle something with Prolog's lazy lists to pull
	//     substitutions from here, then only one call would be needed.
	//   3. could also compile lists of fixed size into a query and use member
	//     to iterate different variable substitutions.
	// Maybe it is not worth it? Also here the different instantiations are processed
	// in parallel as long as enough threads are available. But that could be a bit excessive
	// anyways. for 1. maybe a larger then core-size thread pool would be good as many
	// queries could be active at once such that a scheduler needs to switch between them.
	//
	
	// Get query request from query ID
	auto it = activeQueries_.find(queryID);
	if(it == activeQueries_.end()) {
		spdlog::warn("query {} is not active.", queryID);
		return;
	}
	PrologReasoner::Request &req = it->second;
	
	// create an instance of the input query by applying the substitution.
	std::shared_ptr<Query> query = req.goal->applySubstitution(*bindings.get());
	
	// create a runner for a worker thread
	auto workerGoal = std::shared_ptr<PrologReasoner::Runner>(
		new PrologReasoner::Runner(req.outputStream, query, bindings));
	req.runner.push_back(workerGoal);
	
	// enqueue work
	threadPool_.pushWork(workerGoal);
}


/************************************/
/********* PrologThreadPool *********/
/************************************/

PrologThreadPool::PrologThreadPool(uint32_t maxNumThreads)
: ThreadPool(maxNumThreads)
{}

bool PrologThreadPool::initializeWorker()
{
	// call PL_thread_attach_engine once initially for each worker thread
	if(!PL_thread_attach_engine(NULL)) {
		spdlog::error("failed to attach Prolog engine!");
		return false;
	}
	return true;
}

void PrologThreadPool::finalizeWorker()
{
	// destroy the engine previously bound to this thread
	PL_thread_destroy_engine();
}


/************************************/
/****** PrologReasoner::Runner ******/
/************************************/

PrologReasoner::Runner::Runner(
	const std::shared_ptr<QueryResultStream::Channel> &outputStream,
	const std::shared_ptr<Query> &qa_goal,
	const SubstitutionPtr &bindings)
: ThreadPool::Runner(),
  outputStream_(outputStream),
  qa_goal_(qa_goal),
  pl_goal_(qa_goal),
  bindings_(bindings)
{
}

PrologReasoner::Runner::Runner(
	const std::shared_ptr<QueryResultStream::Channel> &outputStream,
	const std::shared_ptr<Query> &qa_goal)
: Runner(outputStream, qa_goal, QueryResultStream::bos())
{
}

void PrologReasoner::Runner::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;
	
	// the exception risen by the Prolog engine, if any
	term_t exceptionTerm = (term_t)0;
	// open a Prolog query.
	qid_t qid = PL_open_query(
		// the context module of the goal.
		// @see https://www.swi-prolog.org/pldoc/man?section=foreign-modules
		NULL,
		flags,
		// specifies the predicate
		pl_goal_.pl_predicate(),
		// the first of a vector of term references
		pl_goal_.pl_arguments());
	
	// do the query processing
	while(!hasStopRequest()) {
		// here is where the main work is done
		if(!PL_next_solution(qid)) {
			// read exception, if any
			exceptionTerm = PL_exception(qid);
			break;
		}
		// handle stop request
		if(hasStopRequest()) break;
		
		// create substitution mapping from variables to terms.
		// first copy the input bindings, i.e. the variable substitutions that
		// were applied to the input query for this particular runner.
		// TODO: it would be more clean to do this copying of the input bindings
		// centrally such that it does not need to be duplicated for each reasoner.
		auto solution = std::shared_ptr<QueryResult>(new QueryResult(*bindings_.get()));
		
		// second add any additional substitutions to the solution that the
		// Prolog engine could find during query evaluation.
		// NOTE: pl_goal_.vars() maps variable names to term_t references
		//       that also appear in the query given to PL_open_query
		for(const auto& kv: pl_goal_.vars()) {
			solution->set(kv.first, PrologQuery::constructTerm(kv.second));
		}
		
		// push the solution into the output stream
		outputStream_->push(solution);
	}
	
	// free up resources
	PL_close_query(qid);
	
	if(exceptionTerm == (term_t)0) {
		// normal termination
	}
	else {
		// there was an exception
		// TODO: think about if blackboard should handle such exceptions
		char *except_msg = NULL;
		if(PL_get_chars(exceptionTerm, &except_msg, CVT_WRITE | BUF_RING)) {
			spdlog::error("query evaluation failed: {}", except_msg);
		}
		// note that this also free's exception term
		PL_clear_exception();
	}
}

void PrologReasoner::Runner::stop(bool wait)
{
	// TODO: is there a way to terminate above PL_next_solution call in case
	//       runner received a stop request here?
	//     - maybe PL_close_query can be called from here? but not sure about it.
	// wait if requested
	ThreadPool::Runner::stop(wait);
}

