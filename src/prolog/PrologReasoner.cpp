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
  reasonerIDTerm_(std::make_shared<StringTerm>(reasonerID)),
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
	// toggle on flag
	isInitialized_ = true;

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
		std::shared_ptr<Term> term     = result->substitution()->get(*termVar);
		std::shared_ptr<Term> varNames = result->substitution()->get(*listVar);
		
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

const functor_t& PrologReasoner::callFunctor()
{
	static const auto prolog_call_f = PL_new_functor(PL_new_atom("prolog_call"), 2);
	return prolog_call_f;
}

bool PrologReasoner::eval(const std::shared_ptr<Predicate> &p,
						  const char *moduleName,
						  bool doTransformQuery)
{
	return !QueryResultStream::isEOS(oneSolution(p, moduleName, doTransformQuery));
}

bool PrologReasoner::eval(std::shared_ptr<const Query> q,
                          const char *moduleName,
                          bool doTransformQuery)
{
    return !QueryResultStream::isEOS(oneSolution(q, moduleName, doTransformQuery));
}


QueryResultPtr PrologReasoner::oneSolution(const std::shared_ptr<Predicate> &goal,
										   const char *moduleName,
										   bool doTransformQuery)
{
	return oneSolution(std::make_shared<Query>(goal), moduleName, doTransformQuery);
}

QueryResultPtr PrologReasoner::oneSolution(const std::shared_ptr<const Query> &goal,
										   const char *moduleName,
										   bool doTransformQuery)
{
	QueryResultPtr result;

	// create an output queue for the query
	auto outputStream = std::make_shared<QueryResultQueue>();
	auto outputChannel = QueryResultStream::Channel::create(outputStream);
	auto queryInstance = std::make_shared<QueryInstance>(goal, outputChannel);
	auto call_f = (doTransformQuery ? callFunctor() : (functor_t)0);
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologReasoner::Runner>(this,
			PrologReasoner::Request(queryInstance, call_f, moduleName ? moduleName : reasonerID_.c_str()),
			true // sendEOS=true
	);

	// TODO: get any exceptions thrown during evaluation, and throw them here instead!
	PrologReasoner::threadPool().pushWork(workerGoal);
	return outputStream->pop_front();
}

std::list<QueryResultPtr> PrologReasoner::allSolutions(const std::shared_ptr<Predicate> &goal,
													   const char *moduleName,
													   bool doTransformQuery)
{
	return allSolutions(std::make_shared<Query>(goal), moduleName, doTransformQuery);
}

std::list<QueryResultPtr> PrologReasoner::allSolutions(const std::shared_ptr<const Query> &goal,
													   const char *moduleName,
													   bool doTransformQuery)
{
	std::list<QueryResultPtr> results;
	QueryResultPtr nextResult;
	
	// create an output queue for the query
	auto outputStream = std::make_shared<QueryResultQueue>();
	auto outputChannel = QueryResultStream::Channel::create(outputStream);
	auto queryInstance = std::make_shared<QueryInstance>(goal, outputChannel);
	auto call_f = (doTransformQuery ? callFunctor() : (functor_t)0);
	// create a runner for a worker thread
	auto workerGoal = std::make_shared<PrologReasoner::Runner>(this,
			PrologReasoner::Request(queryInstance, call_f, moduleName ? moduleName : reasonerID_.c_str()),
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
								const std::shared_ptr<const Query> &uninstantiatedQuery)
{
	// create a request object and store it in a map
	auto *req = new ActiveQuery;
	req->goal = uninstantiatedQuery;
	req->hasReceivedAllInput = false;
	{
		// protect activeQueries_ with request_mutex_
		std::lock_guard<std::mutex> lock(request_mutex_);
		activeQueries_[queryID] = req;
	}
}

void PrologReasoner::runQueryInstance(uint32_t queryID,
									  const std::shared_ptr<QueryInstance> &queryInstance)
{
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

	// create a runner for a worker thread
	// TODO: avoid passing "this" pointer here
	auto workerGoal = std::make_shared<PrologReasoner::Runner>(this,
			Request(queryInstance, callFunctor(), reasonerID_.c_str(), queryID),
			false // sendEOS
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

void PrologReasoner::finishQuery(uint32_t queryID,
								 const std::shared_ptr<QueryResultStream::Channel> &os,
								 bool isImmediateStopRequested)
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
		os->push(QueryResultStream::eos());
	}
	else {
		// push EOS message to indicate to subscribers that no more
		// messages will be published on this channel.
		bool isEmpty; {
			std::lock_guard<std::mutex> scoped_lock(req->mutex);
			isEmpty = req->runner.empty();
		}
		if(isEmpty) {
			// cleanup if no runners are active anymore
			os->push(QueryResultStream::eos());
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
		// TODO: maybe there is a better way to send EOS? it is error prone that every reasoner
		//        has to do it.
		runner->request_.queryInstance->pushEOS();
		delete req;
		{
			// protect activeQueries_ with request_mutex_
			std::lock_guard<std::mutex> lock(request_mutex_);
			activeQueries_.erase(it);
		}
	}
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

PrologReasoner::Runner::Runner(PrologReasoner *reasoner, Request request, bool sendEOS)
: ThreadPool::Runner(),
  reasoner_(reasoner),
  request_(std::move(request)),
  sendEOS_(sendEOS)
{
}

template <class T>
bool PrologReasoner::Runner::createRangeDict(term_t intervalDict, const Range<T> &range)
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
bool PrologReasoner::Runner::createIntervalDict(term_t intervalDict, const FuzzyInterval<T> &i)
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

term_t PrologReasoner::Runner::createContextTerm(term_t solutionScopeVar)
{
	static const auto time_key = PL_new_atom("time");
	static const auto confidence_key = PL_new_atom("confidenceInterval");
	static const auto query_scope_f = PL_new_functor(PL_new_atom("query_scope"), 1);
	static const auto solution_scope_f = PL_new_functor(PL_new_atom("solution_scope"), 1);
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

	return listTerm;
}

term_t PrologReasoner::Runner::createQueryArgumentTerms(PrologQuery &pl_goal, term_t solutionScopeVar)
{
	static const auto reasoner_module_a = PL_new_atom("reasoner_module");
	static const auto b_setval_f = PL_new_functor(PL_new_atom("b_setval"), 2);
	const auto call_f = request_.callFunctor;

	// construct b_setval/2 arguments
	auto setval_args = PL_new_term_refs(2);
	if(!PL_put_atom(setval_args, reasoner_module_a) ||
	   !PL_put_atom_chars(setval_args+1, request_.queryModule))
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
		   !PL_put_term(call_args+1, createContextTerm(solutionScopeVar)) ||
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

void PrologReasoner::Runner::setSolutionScope(std::shared_ptr<QueryResult> &solution, term_t solutionScope)
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

void PrologReasoner::Runner::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;

	KB_DEBUG("Prolog has new query `{}`.", *request_.goal);
	// use the reasoner module as context module for query evaluation
	module_t ctx_module = PL_new_module(PL_new_atom(request_.queryModule));
	// the exception risen by the Prolog engine, if any
	auto pl_exception = (term_t)0;

	// construct two arguments: `b_setval(reasoner_module, $request_.queryModule)` and `$request_.goal`
	PrologQuery pl_goal(request_.goal);
	auto solution_scope  = PL_new_term_ref();
	auto query_args = createQueryArgumentTerms(pl_goal, solution_scope);
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
			// TODO: what about var-var substitutions? they might need some special handling...
			solution->substitute(Variable(kv.first), PrologQuery::constructTerm(kv.second));
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
	// TODO: check that there are no race conditions for using this pointer here.
	//   the destructor of PrologReasoner attempts to stop runner, it should be double checked
	//   if it is ok already
	reasoner_->finishRunner(request_.queryID, this);

	// make sure EOS is published on output stream
	// TODO: can be done in QueryInstance somehow? why does the flag exist?
	//   - only for the case oneSolution/allSolutions is called directly
	if(sendEOS_) request_.queryInstance->pushEOS();
	// throw error if Prolog evaluation has caused an exception.
	if(exceptionTerm) throw QueryError(*request_.goal, *exceptionTerm);
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
		output.push_back(solution->substitution()->get(*xunit_var));
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
