/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <memory>
#include <filesystem>
#include <sstream>
#include <utility>
#include <gtest/gtest.h>

#include <knowrob/knowrob.h>
#include <knowrob/logging.h>
#include <knowrob/prolog/PrologReasoner.h>
#include <knowrob/prolog/logging.h>
#include <knowrob/prolog/algebra.h>

using namespace knowrob;

bool PrologReasoner::isInitialized_ = false;

PrologReasoner::PrologReasoner(std::string reasonerID)
: reasonerID_(std::move(reasonerID)),
  reasonerIDTerm_(std::make_shared<StringTerm>(reasonerID_)),
  hasRDFData_(false)
{
	addDataFileHandler(PrologDataFile::PROLOG_FORMAT, [this]
			(const DataFilePtr &dataFile){ return consult(dataFile->path()); });
	// TODO: define "rdf-xml" constant
	addDataFileHandler("rdf-xml", [this]
			(const DataFilePtr &dataFile){ return load_rdf_xml(dataFile->path()); });
}

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

std::filesystem::path PrologReasoner::getPrologPath(const std::filesystem::path &filePath)
{
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	if(!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				projectPath / "src" / filePath,
				projectPath / "src" / "prolog" / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for(const auto &p : possiblePaths) {
			if(exists(p)) return p;
		}
	}
	return filePath;
}

std::filesystem::path PrologReasoner::getResourcePath(const std::filesystem::path &filePath)
{
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	// TODO: redundant with above

	if(!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for(const auto &p : possiblePaths) {
			if(exists(p)) return p;
		}
	}
	return filePath;
}

PrologThreadPool& PrologReasoner::threadPool()
{
	// make sure PL_initialise was called
	initializeProlog();
	// a thread pool shared among all PrologReasoner instances
	static PrologThreadPool threadPool_(std::thread::hardware_concurrency());
	return threadPool_;
}

void PrologReasoner::initializeProlog() {
	if(isInitialized_) return;
	static int pl_ac = 0;
	static char *pl_av[5];
	pl_av[pl_ac++] = getNameOfExecutable();
	// '-g true' is used to suppress the welcome message
	pl_av[pl_ac++] = (char *) "-g";
	pl_av[pl_ac++] = (char *) "true";
	// Inhibit any signal handling by Prolog
	pl_av[pl_ac++] = (char *) "--signals=false";
	pl_av[pl_ac]   = nullptr;
	PL_initialise(pl_ac, pl_av);
	KB_DEBUG("Prolog has been initialized.");
}

bool PrologReasoner::initializeGlobalPackages()
{
	// load some default code into user module.  e.g. the extended module syntax etc
	return consult(std::filesystem::path("prolog") / "__init__.pl", "user", false);
}

bool PrologReasoner::loadConfiguration(const ReasonerConfiguration &cfg)
{
	if(!isInitialized_) {
		// call PL_initialize
		initializeProlog();
		// register some foreign predicates, i.e. cpp functions that are used to evaluate predicates.
		// note: the predicates are loaded into module "user"
		// TODO: find a way class member functions can be used as foreign predicates. problem is the
		//   object pointer `this`.
		PL_register_foreign("log_message", 2, (pl_function_t)pl_log_message2, 0);
		PL_register_foreign("log_message", 4, (pl_function_t)pl_log_message4, 0);
		PL_register_extensions_in_module("algebra", algebra_predicates);
		// auto-load some files into "user" module
		initializeGlobalPackages();
		// toggle on flag
		isInitialized_ = true;
	}

	// load properties into the reasoner module.
	// this is needed mainly for the `reasoner_setting/2` that provides reasoner instance specific settings.
	for(auto &pair : cfg.settings) {
		setReasonerSetting(pair.first, pair.second);
	}
	// load reasoner default packages. this is usually the code that implements the reasoner.
	initializeDefaultPackages();

	// load rules and facts
	for(auto &dataFile : cfg.dataFiles) {
		loadDataFile(dataFile);
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

	/*
	if(cfg.get<bool>("semweb:enable", false)) {
		// TODO: maybe a more general handling of data stored "somewhere else" should
		//  be provided
		// TODO: not nice adding this for all reasoner instances here. better require configuration
		//   for adding triple predicate definition into reasoner module
		// load rdf predicates into the reasoner module.
		// this includes triple/3, instance_of/2, etc.
		static auto rdf_init_f =
				std::make_shared<PredicateIndicator>("reasoner_rdf_init", 1);
		eval(std::make_shared<Predicate>(Predicate(rdf_init_f, { reasonerIDTerm_ })), nullptr, false);
	}
	 */

	return true;
}

bool PrologReasoner::setReasonerSetting(const TermPtr &key, const TermPtr &valueString)
{
	static auto set_setting_f =
			std::make_shared<PredicateIndicator>("reasoner_set_setting", 3);
	return eval(std::make_shared<Predicate>(
			Predicate(set_setting_f, { reasonerIDTerm_, key, valueString })),
			nullptr, false);
}


bool PrologReasoner::consult(const std::shared_ptr<FactBase> &factBase)
{
	KB_WARN("Prolog.consult(FactBase) not implemented");
	return false;
}

bool PrologReasoner::consult(const std::shared_ptr<RuleBase> &ruleBase)
{
	KB_WARN("Prolog.consult(RuleBase) not implemented");
	return false;
}

bool PrologReasoner::consult(const std::filesystem::path &prologFile,
							 const char *contextModule,
							 bool doTransformQuery)
{
	static auto consult_f = std::make_shared<PredicateIndicator>("consult", 1);
	auto path = getPrologPath(prologFile);
	auto arg = std::make_shared<StringTerm>(path.native());
	return eval(std::make_shared<Predicate>(Predicate(consult_f, { arg })), contextModule, doTransformQuery);
}

bool PrologReasoner::load_rdf_xml(const std::filesystem::path &rdfFile)
{
	static auto consult_f = std::make_shared<PredicateIndicator>("load_rdf_xml", 2);
	auto path = getResourcePath(rdfFile);
	auto arg0 = std::make_shared<StringTerm>(path.native());
	// remember that an rdf file was loaded
	hasRDFData_ = true;
	return eval(std::make_shared<Predicate>(Predicate(consult_f, { arg0, reasonerIDTerm_ })));
}


bool PrologReasoner::assertFact(const std::shared_ptr<Predicate> &fact)
{
	static auto assert_f = std::make_shared<PredicateIndicator>("assertz", 1);
	return eval(std::make_shared<Predicate>(Predicate(assert_f, { fact })));
}

bool PrologReasoner::isCurrentPredicate(const PredicateIndicator &indicator)
{
	static auto current_predicate_f = std::make_shared<PredicateIndicator>("current_predicate", 1);
	// TODO: could be cached, or initially loaded into a set
	// NOTE: current_predicate includes all predicates loaded into the reasoner module,
	//       not only the ones defined in it. e.g. builtins like `is/2` are included
	//       for each PrologReasoner instance.
	return eval(std::make_shared<Predicate>(Predicate(
				current_predicate_f, { indicator.toTerm() })));
}

std::shared_ptr<Term> PrologReasoner::readTerm(const std::string &queryString)
{
	static std::shared_ptr<Variable> termVar(new Variable("TermFromAtom"));
	static std::shared_ptr<Variable> listVar(new Variable("Vars"));
	static std::shared_ptr<ListTerm> opts(new ListTerm({
		std::make_shared<Predicate>(Predicate("variable_names", {listVar}))
	}));
	
	auto termAtom = std::make_shared<StringTerm>(queryString);
	// run a query
	auto result = oneSolution(std::make_shared<Predicate>(Predicate(
				"read_term_from_atom", { termAtom, termVar, opts })), nullptr, false);
	
	if(QueryResultStream::isEOS(result)) {
		return BottomTerm::get();
	}
	else {
		std::shared_ptr<Term> term     = result->get(*termVar);
		std::shared_ptr<Term> varNames = result->get(*listVar);
		
		if(varNames->type() == TermType::LIST) {
			auto *l = (ListTerm*)varNames.get();
			if(l->elements().empty()) {
				return term;
			}
			
			Substitution s;
			std::map<std::string, Variable*> varNames0;
			for(const auto &x : l->elements()) {
				// each element in the list has the form '='(VarName,Var)
				auto *p = (Predicate*)x.get();
				auto *varName = (StringTerm*)(p->arguments()[0].get());
				auto *var = (Variable*)(p->arguments()[1].get());
				
				s.set(*var, std::make_shared<Variable>(varName->value()));
			}
			
			auto *p0 = (Predicate*)term.get();
			return p0->applySubstitution(s);
		}
		else {
			KB_WARN("something went wrong");
			return term;
		}
	}
}

bool PrologReasoner::eval(const std::shared_ptr<Predicate> &p,
						  const char *moduleName,
						  bool doTransformQuery)
{
	return !QueryResultStream::isEOS(oneSolution(p, moduleName, doTransformQuery));
}

std::shared_ptr<QueryResult> PrologReasoner::oneSolution(const std::shared_ptr<Predicate> &goal,
														 const char *moduleName,
														 bool doTransformQuery)
{
	return oneSolution(std::make_shared<Query>(goal), moduleName, doTransformQuery);
}

std::shared_ptr<QueryResult> PrologReasoner::oneSolution(const std::shared_ptr<Query> &goal,
														 const char *moduleName,
														 bool doTransformQuery)
{
	std::shared_ptr<QueryResult> result;

	// create an output queue for the query
	auto outputStream = std::make_shared<QueryResultQueue>();
	auto outputChannel = QueryResultStream::Channel::create(outputStream);
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologReasoner::Runner>(this,
			PrologReasoner::Request(doTransformQuery ? transformQuery(goal) : goal,
									moduleName ? moduleName : reasonerID_.c_str()),
			outputChannel,
			true // sendEOS=true
	);

	// TODO: get any exceptions thrown during evaluation, and throw them here instead!
	PrologReasoner::threadPool().pushWork(workerGoal);
	return outputStream->pop_front();
}

std::list<std::shared_ptr<QueryResult>> PrologReasoner::allSolutions(const std::shared_ptr<Predicate> &goal,
																	 const char *moduleName,
																	 bool doTransformQuery)
{
	return allSolutions(std::make_shared<Query>(goal), moduleName, doTransformQuery);
}

std::list<std::shared_ptr<QueryResult>> PrologReasoner::allSolutions(const std::shared_ptr<Query> &goal,
																	 const char *moduleName,
																	 bool doTransformQuery)
{
	std::list<std::shared_ptr<QueryResult>> results;
	std::shared_ptr<QueryResult> nextResult;
	
	// create an output queue for the query
	auto outputStream = std::make_shared<QueryResultQueue>();
	auto outputChannel = QueryResultStream::Channel::create(outputStream);
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologReasoner::Runner>(this,
			PrologReasoner::Request(doTransformQuery ? transformQuery(goal) : goal,
									moduleName ? moduleName : reasonerID_.c_str()),
			outputChannel,
			true // sendEOS=true
	);
	
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
	auto *req = new ActiveQuery;
	req->outputStream = outputStream;
	req->goal = transformQuery(goal);
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
	ActiveQueryMap::iterator it; {
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		it = activeQueries_.find(queryID);
		if(it == activeQueries_.end()) {
			KB_WARN("query {} is not active.", queryID);
			return;
		}
	}
	PrologReasoner::ActiveQuery *req = it->second;
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
	ActiveQueryMap::iterator it; {
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		
		it = activeQueries_.find(queryID);
		if(it == activeQueries_.end()) {
			// query is not active
			return;
		}
	}
	PrologReasoner::ActiveQuery *req = it->second;
	
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
	PrologReasoner::ActiveQuery *req; {
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
	auto workerGoal = std::make_shared<PrologReasoner::Runner>(this,
		PrologReasoner::Request(query, reasonerID_.c_str(), queryID),
		req->outputStream,
		false, // sendEOS
		bindings
	);

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
/*********** unit testing ***********/
/************************************/

std::list<TermPtr> PrologReasoner::runTests(const std::string &target)
{
	static const auto xunit_indicator =
			std::make_shared<PredicateIndicator>("xunit_term",1);
	static const auto xunit_var = std::make_shared<Variable>("Term");
	static const auto silent_flag = std::make_shared<StringTerm>("silent");

	auto solutions = allSolutions(std::make_shared<Predicate>(Predicate(
			"test_and_report", {
				// unittest target
				std::make_shared<StringTerm>(target),
				// options
				std::make_shared<ListTerm>(ListTerm({
					std::make_shared<Predicate>(Predicate(xunit_indicator,{xunit_var})),
					silent_flag
				}))
			})), nullptr, false);

	std::list<TermPtr> output;
	for(auto &solution : solutions) {
		output.push_back(solution->get(*xunit_var));
	}
	return output;
}

void PrologTestsBase::runPrologTests(
		const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
		const std::string &target) {
	bool hasResult = false;
	int numTests = 0;
	int numFailedTests = 0;

	for(const auto &t : reasoner->runTests(PrologReasoner::getPrologPath(target))) {
		hasResult = true;
		numTests += 1;

		// each result is a term element/3
		ASSERT_EQ(t->type(), TermType::PREDICATE);
		auto *pElem = (Predicate*)t.get();
		ASSERT_EQ(pElem->indicator().functor(), "element");
		ASSERT_EQ(pElem->indicator().arity(), 3);
		ASSERT_EQ(*(pElem->arguments()[0]), StringTerm("testcase"));

		// the second argument has the form:
		// [name=::string, file=::string, line=::int, time=::double]
		OptionList trace(pElem->arguments()[1]);
		ASSERT_TRUE(trace.contains("file"));
		ASSERT_TRUE(trace.contains("line"));
		ASSERT_TRUE(trace.contains("name"));
		const auto &name = trace.getString("name","");
		const char *file = trace.getString("file","").c_str();
		const long line = trace.getLong("line",0);

		// the third argument is a list of failures.
		auto *failureList = (ListTerm*) pElem->arguments()[2].get();
		if(!failureList->isNIL()) {
			numFailedTests += 1;
		}
		for(const auto &failureTerm : (*failureList)) {
			ASSERT_EQ(failureTerm->type(), TermType::PREDICATE);
			// each failure is a term element/3
			auto *errElem = (Predicate*)failureTerm.get();
			ASSERT_EQ(errElem->indicator().functor(), "element");
			ASSERT_EQ(errElem->indicator().arity(), 3);
			ASSERT_EQ(*(errElem->arguments()[0]), StringTerm("failure"));

			OptionList errOpts(errElem->arguments()[1]);
			ASSERT_TRUE(errOpts.contains("type"));
			ASSERT_TRUE(errOpts.contains("message"));
			auto message = errOpts.getString("type","") + ": " + errOpts.getString("message","");

			// the second argument has the form: [ type=error|failure, message='...' ]
			// the third argument is the exact same message, which is a bit strange.
			//ADD_FAILURE_AT(file,line) << message;
			GTEST_MESSAGE_AT_(file, line, ("test: " + name).c_str(), \
							  testing::TestPartResult::kNonFatalFailure) << message;
		}
	}
	EXPECT_TRUE(hasResult);
	KB_INFO1(target.c_str(), 1, "{}/{} tests succeeded.", (numTests-numFailedTests), numTests);
}

class PrologTestsCore: public PrologTests<knowrob::PrologReasoner> {
protected:
	static std::string getPath(const std::string &filename)
	{ return std::filesystem::path("prolog") / filename; }
};

TEST_F(PrologTestsCore, semweb)	{ runTests(getPath("semweb.pl")); }

/************************************/
/********* PrologDataFile *********/
/************************************/

const std::string PrologDataFile::PROLOG_FORMAT="prolog";

PrologDataFile::PrologDataFile(const std::string &path)
: DataFile(path,PROLOG_FORMAT)
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
	if(PL_thread_attach_engine(nullptr) < 0) {
		// `-1` indicates an error, and `-2` that Prolog is compiled without multithreading support
		KB_ERROR("Failed to attach Prolog engine to current thread!");
		return false;
	}
	else {
		// if PL_thread_attach_engine()>0, then the Prolog ID for the thread was returned
		KB_DEBUG("Attached Prolog engine to current thread.");
		return true;
	}
}

void PrologThreadPool::finalizeWorker()
{
	// destroy the engine previously bound to this thread
	PL_thread_destroy_engine();
	KB_ERROR("destroyed Prolog engine");
}


/************************************/
/****** PrologReasoner::Runner ******/
/************************************/

PrologReasoner::Runner::Runner(
		PrologReasoner *reasoner,
		PrologReasoner::Request request,
		const std::shared_ptr<QueryResultStream::Channel> &outputStream,
		bool sendEOS,
		const SubstitutionPtr &bindings)
: ThreadPool::Runner(),
  reasoner_(reasoner),
  request_(std::move(request)),
  outputStream_(outputStream),
  sendEOS_(sendEOS),
  bindings_(bindings)
{
}

void PrologReasoner::Runner::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;
	static const auto b_setval_f = PL_new_functor(PL_new_atom("b_setval"), 2);
	static const auto reasoner_module_a = PL_new_atom("reasoner_module");

	KB_DEBUG("Prolog has new query `{}`.", *request_.goal);
	// use the reasoner module as context module for query evaluation
	module_t ctx_module = PL_new_module(PL_new_atom(request_.queryModule));
	// the exception risen by the Prolog engine, if any
	auto pl_exception = (term_t)0;
	// construct two arguments: `b_setval(reasoner_module, $request_.queryModule)` and `$request_.goal`
	// TODO: argument vector can be created once for each thread (not once per query as here), then here only
	//       `PL_put_term(query_args+1, pl_goal.pl_query())` needs to be called.
	PrologQuery pl_goal(request_.goal);
	auto query_args  = PL_new_term_refs(2);
	auto setval_args = PL_new_term_refs(2);
	if(!PL_put_atom(setval_args, reasoner_module_a) ||
	   !PL_put_atom_chars(setval_args+1, request_.queryModule) ||
	   !PL_cons_functor_v(query_args, b_setval_f, setval_args) ||
	   !PL_put_term(query_args+1, pl_goal.pl_query()))
	{
		KB_WARN("Failed to create argument vector for query `{}`", *request_.goal);
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
		// create substitution mapping from variables to terms.
		// first copy the input bindings, i.e. the variable substitutions that
		// were applied to the input query for this particular runner.
		// TODO: it would be more clean to do this copying of the input bindings
		//   centrally such that it does not need to be duplicated for each reasoner.
		auto solution = std::make_shared<QueryResult>(*bindings_);

		// second add any additional substitutions to the solution that the
		// Prolog engine could find during query evaluation.
		// NOTE: pl_goal_.vars() maps variable names to term_t references
		//       that also appear in the query given to PL_open_query
		for(const auto& kv: pl_goal.vars()) {
			solution->set(Variable(kv.first), PrologQuery::constructTerm(kv.second));
		}
		
		// push the solution into the output stream
		outputStream_->push(solution);
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
	// TODO: check that there are no race conditions for using this pointer here.
	//   the destructor of PrologReasoner attempts to stop runner, it should be double checked
	//   if it is ok already
	reasoner_->finishRunner(request_.queryID, this);

	// make sure EOS is published on output stream
	if(sendEOS_) outputStream_->push(QueryResultStream::eos());
	// throw error if Prolog evaluation has caused an exception.
	if(exceptionTerm) throw QueryError(*request_.goal, *exceptionTerm);
}
