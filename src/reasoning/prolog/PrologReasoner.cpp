/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <sstream>
#include <knowrob/logging.h>
#include <knowrob/reasoning/prolog/PrologReasoner.h>

using namespace knowrob;


void PrologReasoner::initialize(int argc, char** argv) {
	static bool isInitialized = false;
	static int pl_ac = 0;
	static char *pl_av[5];
	
	if(isInitialized) return;
	isInitialized = true;
	
	pl_av[pl_ac++] = argv[0];
	// '-g true' is used to suppress the welcome message
	pl_av[pl_ac++] = (char *) "-g";
	pl_av[pl_ac++] = (char *) "true";
	// Inhibit any signal handling by Prolog
	// TODO use version macro -nosignals needed for old version
	// pl_av[pl_ac++] = (char *) "-nosignals";
	pl_av[pl_ac++] = (char *) "--signals=false";
	// Limit the combined size of the Prolog stacks to the indicated size.
	//pl_av[pl_ac++] = (char *) "--stack_limit=32g";
	// Limit for the table space.
	// This is where tries holding memoized11 answers for tabling are stored.
	//pl_av[pl_ac++] = (char *) "--table_space=32g";
	//pl_av[pl_ac++] = (char *) "-G256M";
	pl_av[pl_ac] = NULL;
	
	PL_initialise(pl_ac, pl_av);
}


PrologThreadPool& PrologReasoner::threadPool()
{
	// a thread pool shared among all PrologReasoner instances
	static PrologThreadPool threadPool_(std::thread::hardware_concurrency());
	return threadPool_;
}


PrologReasoner::PrologReasoner()
{}

PrologReasoner::~PrologReasoner()
{
	for(auto &pair : activeQueries_) {
		for(auto &x : pair.second->runner) {
			// TODO: not sure if this is safe. is pair removed from map by calling stop()?
			//       and does this interact badly with the outer loop here?
			x->stop(true);
		}
	}
	activeQueries_.clear();
}


bool PrologReasoner::initialize(const ReasonerConfiguration &cfg)
{
	// load rules and facts
	for(auto &dataFile : cfg.dataFiles) {
		consult(dataFile);
	}
	// TODO: support synchronization with data sources.
	//      - when facts are asserted into EDB, also assert into PrologEngine
	//      - support writing facts asserted from PrologEngine into EDB?
	//           i.e. when assert is called in the PrologEngine.
	for(auto &factBase : cfg.factBases) {
		consult(factBase);
	}
	for(auto &ruleBase : cfg.ruleBases) {
		consult(ruleBase);
	}
	return true;
}


bool PrologReasoner::consult(const std::shared_ptr<DataFile> &dataFile)
{
	// TODO: ensure file has the right format?
	//	- could also have importer for certain formats that cannot be loaded natively
	return consult(dataFile->path());
}

bool PrologReasoner::consult(const std::string &prologFile)
{
	// TODO: use Prolog modules to encapsulate reasoner configurations
	return !QueryResultStream::isEOS(oneSolution(std::make_shared<Query>(
		std::shared_ptr<Predicate>(new Predicate(
			"consult", { std::make_shared<StringTerm>(prologFile) }
		))
	)));
}

bool PrologReasoner::consult(const std::shared_ptr<FactBase> &factBase)
{
	KB_WARN("consult(FactBase) not implemented");
	return false;
}

bool PrologReasoner::consult(const std::shared_ptr<RuleBase> &ruleBase)
{
	KB_WARN("consult(RuleBase) not implemented");
	return false;
}


bool PrologReasoner::canReasonAbout(const PredicateIndicator &predicate)
{
	// TODO: could be cached, or initially loaded into a set
	return !QueryResultStream::isEOS(oneSolution(std::make_shared<Query>(
		std::shared_ptr<Predicate>(new Predicate(
			"current_functor", {
				std::make_shared<StringTerm>(predicate.functor()),
				std::make_shared<Integer32Term>(predicate.arity())
			}
		))
	)));
}

bool PrologReasoner::assertFact(const std::shared_ptr<Predicate> &fact)
{
	return !QueryResultStream::isEOS(oneSolution(std::make_shared<Query>(
		std::shared_ptr<Predicate>(new Predicate(
			"user:assertz", { fact }
		))
	)));
}

std::shared_ptr<Term> PrologReasoner::readTerm(const std::string &queryString)
{
	static std::shared_ptr<Variable> termVar(new Variable("TermFromAtom"));
	static std::shared_ptr<Variable> listVar(new Variable("Vars"));
	static std::shared_ptr<ListTerm> opts(new ListTerm({
		std::shared_ptr<Predicate>(new Predicate("variable_names", {listVar}))
	}));
	
	auto termAtom = std::shared_ptr<StringTerm>(new StringTerm(queryString));
	// run a query
	auto result = oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate(
			"read_term_from_atom", { termAtom, termVar, opts }
		))
	)));
	
	if(QueryResultStream::isEOS(result)) {
		return BottomTerm::get();
	}
	else {
		std::shared_ptr<Term> term     = result->get(*termVar.get());
		std::shared_ptr<Term> varNames = result->get(*listVar.get());
		
		if(varNames->type() == TermType::LIST) {
			ListTerm *l = (ListTerm*)varNames.get();
			if(l->elements().empty()) {
				return term;
			}
			
			Substitution s;
			std::map<std::string, Variable*> varNames0;
			for(const auto &x : l->elements()) {
				// each element in the list has the form '='(VarName,Var)
				Predicate *p = (Predicate*)x.get();
				StringTerm *varName = (StringTerm*)(p->arguments()[0].get());
				Variable   *var = (Variable*)(p->arguments()[1].get());
				
				s.set(*var, std::make_shared<Variable>(varName->value()));
			}
			
			Predicate *p0 = (Predicate*)term.get();
			return p0->applySubstitution(s);
		}
		else {
			KB_WARN("something went wrong");
			return term;
		}
	}
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
	workerGoal->reasoner = this;
	
	// TODO: get any exceptions thrown during evaluation, and throw them here instead!
	PrologReasoner::threadPool().pushWork(workerGoal);
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
	workerGoal->reasoner = this;
	
	// assign the goal to a worker thread
	PrologReasoner::threadPool().pushWork(workerGoal);
	// wait until work is done, and push EOS
	workerGoal->join();
	outputChannel->push(QueryResultStream::eos());
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
	auto *req = new Request;
	req->outputStream = outputStream;
	req->goal = goal;
	req->hasReceivedAllInput = false;
	{
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		activeQueries_[queryID] = req;
	}
}

void PrologReasoner::finishQuery(uint32_t queryID, bool isImmediateStopRequested)
{
	// Get query request from query ID
	RequestMap::iterator it; {
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		it = activeQueries_.find(queryID);
		if(it == activeQueries_.end()) {
			KB_WARN("query {} is not active.", queryID);
			return;
		}
	}
	PrologReasoner::Request *req = it->second;
	req->hasReceivedAllInput = true;
	
	if(isImmediateStopRequested) {
		// instruct all active query runners to stop.
		// This basically attempts to gracefully stop them just by toggling
		// on a flag that makes the runner break out its loop in the next iteration,
		// or right away if the thread is sleeping.
		// note: no need to wait for the runner to be stopped here. it could be the runner
		// needs more time to terminate, or even never terminates.
		{
			std::lock_guard<std::mutex> scoped_lock(req->mutex);
			for(auto &x : req->runner) {
				x->stop(false);
			}
		}
		req->outputStream->push(QueryResultStream::eos());
	}
	else {
		// push EOS message to indicate to subscribers that no more
		// messages will be published on this channel.
		bool isEmpty; {
			std::lock_guard<std::mutex> scoped_lock(req->mutex);
			isEmpty = req->runner.empty();
		}
		if(isEmpty) {
			req->outputStream->push(QueryResultStream::eos());
			delete req;
			{
				// protect activeQueries_ with request_mutex_
				std::lock_guard<std::mutex> lock(request_mutex_);
				activeQueries_.erase(it);
			}
		}
	}
}

void PrologReasoner::finishRunner(uint32_t queryID, PrologReasoner::Runner *runner)
{
	// Get query request from query ID
	RequestMap::iterator it; {
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		
		it = activeQueries_.find(queryID);
		if(it == activeQueries_.end()) {
			// query is not active
			return;
		}
	}
	PrologReasoner::Request *req = it->second;
	
	bool isRequestFinished; {
		std::lock_guard<std::mutex> lock(req->mutex);
		// remove runner from list in the request
		req->runner.erase(runner->requestIterator);
		// request is finished when no more runner is active, and no more input to process
		isRequestFinished = (req->runner.empty() && req->hasReceivedAllInput);
	}
	if(isRequestFinished) {
		// send EOF and delete request if this was the last runner
		req->outputStream->push(QueryResultStream::eos());
		delete req;
		{
			// protect activeQueries_ with request_mutex_
			std::lock_guard<std::mutex> lock(request_mutex_);
			activeQueries_.erase(it);
		}
	}
}

void PrologReasoner::pushSubstitution(uint32_t queryID, const SubstitutionPtr &bindings)
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
	PrologReasoner::Request *req; {
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		auto it = activeQueries_.find(queryID);
		if(it == activeQueries_.end()) {
			KB_WARN("cannot push substitution for inactive query {}.", queryID);
			return;
		}
		req = it->second;
	}
	
	// create an instance of the input query by applying the substitution.
	std::shared_ptr<Query> query = req->goal->applySubstitution(*bindings.get());
	
	// create a runner for a worker thread
	auto workerGoal = std::shared_ptr<PrologReasoner::Runner>(
		new PrologReasoner::Runner(req->outputStream, query, bindings));
	workerGoal->reasoner = this;
	workerGoal->queryID = queryID;
	// add runner to list of runners associated to this request
	{
		std::lock_guard<std::mutex> lock(req->mutex);
		req->runner.push_back(workerGoal);
		workerGoal->requestIterator = req->runner.end();
		--workerGoal->requestIterator;
	}
	
	// add work to queue
	PrologReasoner::threadPool().pushWork(workerGoal);
}

/************************************/
/********* PrologDataFile *********/
/************************************/

PrologDataFile::PrologDataFile(const std::string &path)
: DataFile(path)
{}

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
		KB_ERROR("failed to attach Prolog engine!");
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
	const std::shared_ptr<Query> &goal,
	const SubstitutionPtr &bindings)
: ThreadPool::Runner(),
  outputStream_(outputStream),
  goal_(goal),
  bindings_(bindings)
{
}

PrologReasoner::Runner::Runner(
	const std::shared_ptr<QueryResultStream::Channel> &outputStream,
	const std::shared_ptr<Query> &goal)
: Runner(outputStream, goal, QueryResultStream::bos())
{
}

void PrologReasoner::Runner::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;
	
	TermPtr exceptionTerm;
	bool hasException = false;
	
	KB_DEBUG("Prolog has new query.");
	PrologQuery pl_goal(goal_);
	// the exception risen by the Prolog engine, if any
	term_t pl_exception = (term_t)0;
	// open a Prolog query.
	qid_t qid = PL_open_query(
		// the context module of the goal.
		// @see https://www.swi-prolog.org/pldoc/man?section=foreign-modules
		NULL,
		flags,
		// specifies the predicate
		pl_goal.pl_predicate(),
		// the first of a vector of term references
		pl_goal.pl_arguments());
	
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
		
		KB_DEBUG("Prolog has a next solution for query.");
		// create substitution mapping from variables to terms.
		// first copy the input bindings, i.e. the variable substitutions that
		// were applied to the input query for this particular runner.
		// TODO: it would be more clean to do this copying of the input bindings
		// centrally such that it does not need to be duplicated for each reasoner.
		auto solution = std::make_shared<QueryResult>(*bindings_.get());
		
		// second add any additional substitutions to the solution that the
		// Prolog engine could find during query evaluation.
		// NOTE: pl_goal_.vars() maps variable names to term_t references
		//       that also appear in the query given to PL_open_query
		for(const auto& kv: pl_goal.vars()) {
			solution->set(kv.first, PrologQuery::constructTerm(kv.second));
		}
		
		// push the solution into the output stream
		outputStream_->push(solution);
	}
	
	// construct exception term
	if(pl_exception != (term_t)0) {
		hasException = true;
		exceptionTerm = PrologQuery::constructTerm(pl_exception);
		PL_clear_exception();
	}
	
	KB_DEBUG("Prolog query completed.");
	// free up resources
	PL_close_query(qid);
	// notify PrologReasoner about runner being done
	// FIXME: this would cause segfault if reasoner destructor was called before
	reasoner->finishRunner(queryID, this);
	
	// throw error if Prolog evaluation has caused an exception.
	if(hasException) {
		throw QueryError(*goal_.get(), *exceptionTerm.get());
	}
}

void PrologReasoner::Runner::stop(bool wait)
{
	// TODO: is there a way to terminate above PL_next_solution call in case
	//       runner received a stop request here?
	//     - maybe PL_close_query can be called from here? seems unlikely
	//     - *yielding* might be a way to avoid blocking in PL_next_solution
	// wait if requested
	ThreadPool::Runner::stop(wait);
}

