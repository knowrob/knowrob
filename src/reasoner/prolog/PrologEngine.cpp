/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <SWI-Prolog.h>
#include <filesystem>
#include <clocale>
#include <cstdlib>

#include "knowrob/Logger.h"
#include "knowrob/reasoner/prolog/PrologEngine.h"
#include "knowrob/knowrob.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/reasoner/prolog/logging.h"
#include "knowrob/reasoner/prolog/ext/algebra.h"
#include "knowrob/reasoner/prolog/semweb.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/URI.h"

using namespace knowrob;

static const int prologQueryFlags = PL_Q_CATCH_EXCEPTION | PL_Q_NODEBUG;

bool PrologEngine::isPrologInitialized_ = false;

PrologEngine::PrologEngine(uint32_t maxNumThreads)
		: ThreadPool(maxNumThreads) {
	initializeProlog();
}

bool PrologEngine::initializeWorker() {
	// call PL_thread_attach_engine once initially for each worker thread
	if (PL_thread_attach_engine(nullptr) < 0) {
		// `-1` indicates an error, and `-2` that Prolog is compiled without multithreading support
		KB_ERROR("Failed to attach Prolog engine to current thread!");
		return false;
	} else {
		// if PL_thread_attach_engine()>0, then the Prolog ID for the thread was returned
		KB_DEBUG("Attached Prolog engine to current thread.");
		return true;
	}
}

foreign_t pl_rdf_register_namespace2(term_t prefix_term, term_t uri_term) {
	char *prefix, *uri;
	if (PL_get_atom_chars(prefix_term, &prefix) && PL_get_atom_chars(uri_term, &uri)) {
		PrefixRegistry::registerPrefix(prefix, uri);
	}
	return TRUE;
}

foreign_t url_resolve2(term_t t_url, term_t t_resolved) {
	char *url;
	if (PL_get_atom_chars(t_url, &url)) {
		auto resolved = URI::resolve(url);
		return PL_unify_atom_chars(t_resolved, resolved.c_str());
	}
	return false;
}

void PrologEngine::initializeProlog() {
	if (isPrologInitialized_) return;
	// toggle on flag
	isPrologInitialized_ = true;

	// PL_initialise changes the locale. This can have bad consequences.
	// I experienced unit tests for Redland failing because of this!
	// Seems like it is related to parsing of numbers. Setting
	// LC_NUMERIC to "C" before PL_initialise seems to fix the problem.
	setenv("LC_NUMERIC", "C", 1);

	static int pl_ac = 0;
	static char *pl_av[5];
	pl_av[pl_ac++] = getNameOfExecutable();
	// '-g true' is used to suppress the welcome message
	pl_av[pl_ac++] = (char *) "-g";
	pl_av[pl_ac++] = (char *) "true";
	// Inhibit any signal handling by Prolog
	pl_av[pl_ac++] = (char *) "--signals=false";
	pl_av[pl_ac] = nullptr;
	PL_initialise(pl_ac, pl_av);
	KB_DEBUG("Prolog has been initialized.");

	// register some foreign predicates, i.e. cpp functions that are used to evaluate predicates.
	// note: the predicates are loaded into module "user"
	PL_register_foreign("knowrob_register_namespace",
						2, (pl_function_t) pl_rdf_register_namespace2, 0);
	PL_register_foreign("url_resolve",
						2, (pl_function_t) url_resolve2, 0);
	PL_register_foreign("log_message", 2, (pl_function_t) prolog::log_message2, 0);
	PL_register_foreign("log_message", 4, (pl_function_t) prolog::log_message4, 0);
	PL_register_extensions_in_module("algebra", prolog::PL_extension_algebra);
	PL_register_extensions_in_module("semweb", prolog::PL_extension_semweb);
	KB_DEBUG("common foreign Prolog modules have been registered.");

	consult(std::filesystem::path("reasoner") / "prolog" / "__init__.pl", "user", false);
	KB_DEBUG("KnowRob __init__.pl has been consulted.");
}

void PrologEngine::finalizeWorker() {
	// destroy the engine previously bound to this thread
	PL_thread_destroy_engine();
	KB_DEBUG("destroyed Prolog engine");
}

void PrologEngine::pushGoal(const std::shared_ptr<ThreadPool::Runner> &goal, const ErrorHandler &errHandler) {
	static PrologEngine prologEngine(std::thread::hardware_concurrency());
	prologEngine.pushWork(goal, errHandler);
}

void PrologEngine::pushGoalAndJoin(const std::shared_ptr<ThreadPool::Runner> &goal) {
	// push goal and wait for termination
	std::exception_ptr excPtr = nullptr;
	pushGoal(goal, [&excPtr](...) { excPtr = std::current_exception(); });
	goal->join();
	// rethrow any exceptions in this thread
	if (excPtr) std::rethrow_exception(excPtr);
}

bool PrologEngine::eval(const GoalFactory &goalFactory) {
	bool hasSolution = false;

	pushGoalAndJoin(std::make_shared<LambdaRunner>(
			[&goalFactory, &hasSolution](const LambdaRunner::StopChecker &) {
				auto goal = goalFactory();
				auto qid = goal.openQuery(prologQueryFlags);
				hasSolution = goal.nextSolution(qid);
				PL_close_query(qid);
			}));

	return hasSolution;
}

std::optional<PrologEngine::Solution> PrologEngine::oneSolution(const GoalFactory &goalFactory) {
	std::optional<Solution> solution;

	pushGoalAndJoin(std::make_shared<LambdaRunner>(
			[&goalFactory, &solution](const LambdaRunner::StopChecker &) {
				auto goal = goalFactory();
				auto qid = goal.openQuery(prologQueryFlags);
				if (goal.nextSolution(qid)) {
					solution = Solution();
					for (auto &var: goal.vars()) {
						solution.value()[var.first.c_str()] = PrologTerm::toKnowRobTerm(var.second);
					}
				}
				PL_close_query(qid);
			}));

	return solution;
}

std::vector<PrologEngine::Solution> PrologEngine::allSolutions(const GoalFactory &goalFactory) {
	std::vector<Solution> solutions;

	pushGoalAndJoin(std::make_shared<LambdaRunner>(
			[&goalFactory, &solutions](const LambdaRunner::StopChecker &hasStopRequest) {
				auto goal = goalFactory();
				auto qid = goal.openQuery(prologQueryFlags);
				while (!hasStopRequest() && goal.nextSolution(qid)) {
					Solution solution;
					for (auto &var: goal.vars()) {
						solution[var.first.c_str()] = PrologTerm::toKnowRobTerm(var.second);
					}
					solutions.push_back(solution);
				}
				PL_close_query(qid);
			}));

	return solutions;
}

void PrologEngine::query(const GoalFactory &goalFactory, const BindingsHandler &callback) {
	pushGoalAndJoin(std::make_shared<LambdaRunner>(
			[&goalFactory, &callback](const LambdaRunner::StopChecker &hasStopRequest) {
				auto goal = goalFactory();
				auto qid = goal.openQuery(prologQueryFlags);
				while (!hasStopRequest() && goal.nextSolution(qid)) {
					auto bindings = std::make_shared<Bindings>();
					for (auto &var: goal.vars()) {
						if (PL_term_type(var.second) == PL_VARIABLE) {
							continue;
						}
						bindings->set(std::make_shared<Variable>(var.first),
									  PrologTerm::toKnowRobTerm(var.second));
					}
					callback(bindings);
				}
				PL_close_query(qid);
			}));
}

bool PrologEngine::consult(const std::filesystem::path &uri, const char *module, bool doTransformQuery) {
	static auto consult_f = "consult";
	auto path = PrologEngine::getPrologPath(uri);

	return PrologEngine::eval([&]() {
		PrologTerm plTerm(consult_f, path.native());
		if (module) plTerm.setModule(module);
		return plTerm;
	});
}

std::filesystem::path PrologEngine::getPrologPath(const std::filesystem::path &filePath) {
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	if (!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				projectPath / "src" / filePath,
				projectPath / "src" / "reasoner" / "prolog" / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for (const auto &p: possiblePaths) {
			if (exists(p)) return p;
		}
	}
	return filePath;
}

std::filesystem::path PrologEngine::getResourcePath(const std::filesystem::path &filePath) {
	static std::filesystem::path projectPath(KNOWROB_SOURCE_DIR);
	static std::filesystem::path installPath(KNOWROB_INSTALL_PREFIX);

	if (!exists(filePath)) {
		auto possiblePaths = {
				projectPath / filePath,
				installPath / "share" / "knowrob" / filePath
		};
		for (const auto &p: possiblePaths) {
			if (exists(p)) return p;
		}
	}
	return filePath;
}
