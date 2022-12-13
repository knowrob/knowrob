/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

// STD
#include <string>
#include <list>
#include <map>
#include <memory>
// KnowRob
#include <knowrob/ThreadPool.h>
#include <knowrob/lang/terms.h>
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/LogicProgramReasoner.h>
#include <knowrob/reasoning/prolog/PrologQuery.h>

namespace knowrob {
	/** A pool of threads with attached Prolog engines.
	 * Prolog threads have their own stacks and only share the Prolog heap:
	 * predicates, records, flags and other global non-backtrackable data.
	 */
	class PrologThreadPool : public ThreadPool {
	public:
		/**
		 * @maxNumThreads maximum number of worker threads.
		 */
		PrologThreadPool(uint32_t maxNumThreads=0);

	protected:
		// Override ThreadPool
		bool initializeWorker();
		
		// Override ThreadPool
		void finalizeWorker();
	};

	/**
	 * A Prolog reasoner that answers queries using SWI Prolog.
	 */
	class PrologReasoner : public LogicProgramReasoner {
	public:
		/**
		 * @initFile the path to a Prolog-encoded file that is initially consulted.
		 */
		PrologReasoner(const std::string &initFile);
		
		~PrologReasoner();
		
		PrologReasoner(const PrologReasoner&) = delete;
		
		static void initialize(int argc, char** argv);
		
		std::shared_ptr<Term> readTerm(const std::string &queryString);

		/**
		 * Consults a Prolog file, i.e. loads facts and rules and executed
		 * directives in the file.
		 * May throw an exception if there is no valid Prolog file at the given path.
		 * @prologFile the local path to the file.
		 */
		bool consult(const std::string &prologFile);
		
		/** Evaluates a query and returns one solution if any.
		 * @return the first solution found, or QueryResultStream::eos().
		 */
		std::shared_ptr<QueryResult> oneSolution(const std::shared_ptr<Query> &goal);
		
		/** Evaluates a query and returns all solutions.
		 * @return list of solutions.
		 */
		std::list<std::shared_ptr<QueryResult>> allSolutions(const std::shared_ptr<Query> &goal);

		// Override LogicProgramReasoner
		bool assertFact(const std::shared_ptr<Predicate> &predicate);

		// Override IReasoner
 		void initialize();

		// Override IReasoner
		bool canReasonAbout(const PredicateIndicator &predicate);

		// Override IReasoner
		void startQuery(uint32_t queryID,
			const std::shared_ptr<QueryResultStream::Channel> &outputStream,
			const std::shared_ptr<Query> &goal);
		
		// Override IReasoner
		void finishQuery(uint32_t queryID,
			bool isImmediateStopRequested);
		
		// Override IReasoner
		void pushSubstitution(uint32_t queryID,
			const SubstitutionPtr &bindings);

	protected:
		PrologThreadPool threadPool_;
		std::string initFile_;
		
		/** A runner that evaluates a Prolog query.
		 */
		class Runner : public ThreadPool::Runner {
		public:
			Runner(const std::shared_ptr<QueryResultStream::Channel> &outputStream,
				const std::shared_ptr<Query> &goal,
				const SubstitutionPtr &bindings);
			
			Runner(const std::shared_ptr<QueryResultStream::Channel> &outputStream,
				const std::shared_ptr<Query> &goal);
			
			// Override Runner
			void stop(bool wait);
			// Override Runner
			void run();
		
		protected:
			std::shared_ptr<QueryResultStream::Channel> outputStream_;
			std::shared_ptr<Query> goal_;
			SubstitutionPtr bindings_;
			
		};
		
		struct Request {
			std::shared_ptr<QueryResultStream::Channel> outputStream;
			std::shared_ptr<Query> goal;
			std::list<std::shared_ptr<PrologReasoner::Runner>> runner;
		};
		std::map<uint32_t, PrologReasoner::Request> activeQueries_;
	};
}

#endif //__KNOWROB_PROLOG_REASONER_H__
