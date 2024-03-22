/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_THREAD_POOL_H_
#define KNOWROB_PROLOG_THREAD_POOL_H_

#include "filesystem"
#include "knowrob/ThreadPool.h"
#include "PrologTerm.h"

namespace knowrob {
	/**
	 * A pool of threads with attached Prolog engines.
	 * Prolog threads have their own stacks and only share the Prolog heap:
	 * predicates, records, flags and other global non-backtrackable data.
	 * NOTE: term_t cannot be created in the main thread and used it in a worker thread!
	 * The term reference will be most likely a variable in the worker thread no matter what
	 * value it has in the main thread.
	 */
	class PrologEngine : public ThreadPool {
	public:
		using ErrorHandler = ThreadPool::ExceptionHandler;
		using GoalFactory = std::function<PrologTerm()>;
		using Solution = std::map<std::string_view, TermPtr>;

		/**
		 * @maxNumThreads maximum number of worker threads.
		 */
		explicit PrologEngine(uint32_t maxNumThreads = 0);

		/**
		 * Must be called before any Prolog library interactions.
		 */
		static void initializeProlog();

		/**
		 * Evaluate a goal in a thread with a Prolog engine.
		 * @param goal the work goal
		 * @return true if the goal succeeded
		 */
		static bool eval(const GoalFactory &goalFactory);

		/**
		 * Evaluate a goal in a thread with a Prolog engine.
		 * @param goal the work goal
		 * @return the first solution found or an empty optional
		 */
		static std::optional<Solution> oneSolution(const GoalFactory &goalFactory);

		/**
		 * Evaluate a goal in a thread with a Prolog engine.
		 * @param goal the work goal
		 * @return all solutions found
		 */
		static std::vector<Solution> allSolutions(const GoalFactory &goalFactory);

		/**
		 * Evaluate a goal in a thread with a Prolog engine.
		 * @param goal the work goal
		 * @param callback a function that handles the solutions
		 */
		static void query(const GoalFactory &goalFactory, const BindingsHandler &callback);

		/**
		 * Consults a Prolog file, i.e. loads facts and rules and executed
		 * directives in the file.
		 * May throw an exception if there is no valid Prolog file at the given path.
		 * @prologFile the local path to the file.
		 * @return true on success
		 */
		static bool consult(const std::filesystem::path &uri, const char *module = {}, bool doTransformQuery = true);

		/**
		 * Run a goal in a worker thread with a Prolog engine.
		 * @param goal the work goal
		 * @param exceptionHandler a function that handles exceptions thrown by the goal
		 */
		static void pushGoal(const std::shared_ptr<ThreadPool::Runner> &goal, const ErrorHandler &errHandler);

		/**
		 * Run a goal in a worker thread with a Prolog engine and wait for termination.
		 * Also rethrow any exceptions in the thread of the caller.
		 * @param goal the work goal
		 */
		static void pushGoalAndJoin(const std::shared_ptr<ThreadPool::Runner> &goal);

		/**
		 * Resolve the path to a Prolog file.
		 * The function attempts to resolve project-relative paths.
		 * @param filename a name or path.
		 * @return a path where the file might be stored
		 */
		static std::filesystem::path getPrologPath(const std::filesystem::path &filename);

		/**
		 * Resolve the path to a resource file.
		 * The function attempts to resolve project-relative paths.
		 * @param filename a name or path.
		 * @return a path where the file might be stored
		 */
		static std::filesystem::path getResourcePath(const std::filesystem::path &filename);

	protected:
		static bool isPrologInitialized_;

		// Override ThreadPool
		bool initializeWorker() override;

		// Override ThreadPool
		void finalizeWorker() override;
	};
}

#define PROLOG_ENGINE_EVAL(term) PrologEngine::eval([&]() { return term; })
#define PROLOG_ENGINE_ONE_SOL(term) PrologEngine::oneSolution([&]() { return term; })
#define PROLOG_ENGINE_ALL_SOL(term) PrologEngine::allSolutions([&]() { return term; })
#define PROLOG_ENGINE_QUERY(term,callback) PrologEngine::query([&]() { return term; }, callback)

#endif //KNOWROB_PROLOG_THREAD_POOL_H_
