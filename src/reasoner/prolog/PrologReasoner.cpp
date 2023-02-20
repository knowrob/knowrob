/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <memory>
#include <filesystem>
#include <utility>

#include "knowrob/knowrob.h"
#include "knowrob/Logger.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"
#include "knowrob/reasoner/prolog/logging.h"
#include "knowrob/reasoner/prolog/algebra.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/Bottom.h"
#include "knowrob/queries/QueryResultQueue.h"
#include "knowrob/rdf/PrefixRegistry.h"

using namespace knowrob;

// make reasoner type accessible
KNOWROB_BUILTIN_REASONER("Prolog", PrologReasoner)

// forward declarations of foreign predicates
foreign_t pl_rdf_register_namespace2(term_t prefix_term, term_t uri_term);
// defined in blackboard.cpp
extern PL_extension qa_predicates[];

bool PrologReasoner::isPrologInitialized_ = false;
bool PrologReasoner::isKnowRobInitialized_ = false;

PrologReasoner::PrologReasoner(std::string reasonerID)
: reasonerID_(std::move(reasonerID)),
  reasonerIDTerm_(std::make_shared<StringTerm>(reasonerID_))
{
    addDataSourceHandler(DataSource::PROLOG_FORMAT, [this]
            (const DataSourcePtr &dataFile) { return consult(dataFile->uri()); });
    addDataSourceHandler(DataSource::RDF_XML_FORMAT, [this]
            (const DataSourcePtr &dataFile) { return load_rdf_xml(dataFile->uri()); });
}

PrologReasoner::~PrologReasoner()
{
    auto queries = activeQueries_;
	for(auto &pair : queries) {
        auto runner = pair.second->runner;
		for(auto &x : runner) {
			x->stop(true);
		}
	}
	activeQueries_.clear();
}

unsigned long PrologReasoner::getCapabilities() const
{
	return CAPABILITY_CONJUNCTIVE_QUERIES |
		   CAPABILITY_DISJUNCTIVE_QUERIES;
}

const functor_t& PrologReasoner::callFunctor()
{
	static const auto reasoner_call_f = PL_new_functor(PL_new_atom("reasoner_call"), 2);
	return reasoner_call_f;
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
	if(isPrologInitialized_) return;
	// toggle on flag
    isPrologInitialized_ = true;

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
	return consult(std::filesystem::path("reasoner") / "prolog" / "__init__.pl",
                   "user", false);
}

bool PrologReasoner::loadConfiguration(const ReasonerConfiguration &cfg)
{
    // call PL_initialize
    initializeProlog();

	if(!isKnowRobInitialized_) {
        isKnowRobInitialized_ = true;
		// register some foreign predicates, i.e. cpp functions that are used to evaluate predicates.
		// note: the predicates are loaded into module "user"
        PL_register_foreign("knowrob_register_namespace",
                            2, (pl_function_t)pl_rdf_register_namespace2, 0);
		PL_register_foreign("log_message", 2, (pl_function_t)pl_log_message2, 0);
		PL_register_foreign("log_message", 4, (pl_function_t)pl_log_message4, 0);
		PL_register_extensions_in_module("algebra", algebra_predicates);
		PL_register_extensions_in_module("user", qa_predicates);
		// auto-load some files into "user" module
		initializeGlobalPackages();

        // register RDF namespaces with Prolog.
        // in particular the ones specified in settings are globally registered with PrefixRegistry.
        static const auto register_prefix_i =
                std::make_shared<PredicateIndicator>("rdf_register_prefix", 3);
        for(auto &pair : rdf::PrefixRegistry::get()) {
            const auto &uri = pair.first;
            const auto &alias = pair.second;
            eval(std::make_shared<Predicate>(Predicate(register_prefix_i, {
                    std::make_shared<StringTerm>(alias),
                    std::make_shared<StringTerm>(uri+"#")
            })), nullptr, false);
        }
	}

	// load properties into the reasoner module.
	// this is needed mainly for the `reasoner_setting/2` that provides reasoner instance specific settings.
	for(auto &pair : cfg.settings) {
		setReasonerSetting(pair.first, pair.second);
	}
	// load reasoner default packages. this is usually the code that implements the reasoner.
	initializeDefaultPackages();

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
	return eval(std::make_shared<Predicate>(Predicate(consult_f, { arg0, reasonerIDTerm_ })));
}

bool PrologReasoner::assertFact(const std::shared_ptr<Predicate> &fact)
{
	static auto assert_f = std::make_shared<PredicateIndicator>("assertz", 1);
	return eval(std::make_shared<Predicate>(Predicate(assert_f, { fact })));
}

std::shared_ptr<PredicateDescription> PrologReasoner::getPredicateDescription(
		const std::shared_ptr<PredicateIndicator> &indicator)
{
	static auto current_predicate_f = std::make_shared<PredicateIndicator>(
			"reasoner_defined_predicate", 2);

	// TODO: rather cache predicate descriptions centrally for all reasoner
	auto it = predicateDescriptions_.find(*indicator);
	if(it != predicateDescriptions_.end()) {
		return it->second;
	}
	else {
		// evaluate query
		auto type_v = std::make_shared<Variable>("Type");
		auto solution = oneSolution(std::make_shared<Predicate>(Predicate(
				current_predicate_f, { indicator->toTerm(), type_v })));

		if(QueryResultStream::isEOS(solution)) {
			// FIXME: this is not safe if files are consulted at runtime
			static std::shared_ptr<PredicateDescription> nullDescr;
			predicateDescriptions_[*indicator] = nullDescr;
			return nullDescr;
		}
		else {
			// read type of predicate
			auto ptype = predicateTypeFromTerm(solution->substitution()->get(*type_v));
			// create a PredicateDescription
			auto newDescr = std::make_shared<PredicateDescription>(indicator, ptype);
			predicateDescriptions_[*indicator] = newDescr;
			return newDescr;
		}
	}
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
				"read_query", { termAtom, termVar, opts })), nullptr, false);
	
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

bool PrologReasoner::eval(const std::shared_ptr<Predicate> &p,
                          const char *moduleName,
                          bool doTransformQuery) {
    return !QueryResultStream::isEOS(oneSolution(p, moduleName, doTransformQuery));
}

bool PrologReasoner::eval(const std::shared_ptr<const Query> &q,
                              const char *moduleName,
                              bool doTransformQuery) {
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
	auto workerGoal = std::make_shared<PrologQueryRunner>(
            this,
            PrologQueryRunner::Request(queryInstance, call_f, moduleName ?
                std::make_shared<StringTerm>(moduleName) : reasonerIDTerm_),
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
	auto workerGoal = std::make_shared<PrologQueryRunner>(
            this,
            PrologQueryRunner::Request(queryInstance, call_f, moduleName ?
                std::make_shared<StringTerm>(moduleName) : reasonerIDTerm_),
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
	auto workerGoal = std::make_shared<PrologQueryRunner>(
            this,
            PrologQueryRunner::Request(queryInstance, callFunctor(), reasonerIDTerm_, queryID),
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

void PrologReasoner::finishRunner(uint32_t queryID, PrologQueryRunner *runner)
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
		// TODO: maybe there is a better way to send EOS? it is error prone that every reasoner has to do it.
		runner->request_.queryInstance->pushEOS();
		delete req;
		{
			// protect activeQueries_ with request_mutex_
			std::lock_guard<std::mutex> lock(request_mutex_);
			activeQueries_.erase(it);
		}
	}
}

std::filesystem::path PrologReasoner::getPrologPath(const std::filesystem::path &filePath)
{
    static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
    static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

    if(!exists(filePath)) {
        auto possiblePaths = {
                projectPath / filePath,
                projectPath / "src" / filePath,
                projectPath / "src" / "reasoner" / "prolog" / filePath,
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

// FOREIGN PREDICATES

foreign_t pl_rdf_register_namespace2(term_t prefix_term, term_t uri_term)
{
    char *prefix, *uri;
    if(PL_get_atom_chars(prefix_term, &prefix) && PL_get_atom_chars(uri_term, &uri)) {
        rdf::PrefixRegistry::get().registerPrefix(prefix, uri);
    }
    return TRUE;
}
