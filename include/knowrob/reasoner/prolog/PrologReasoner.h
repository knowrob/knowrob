/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_REASONER_H_
#define KNOWROB_PROLOG_REASONER_H_

// STD
#include <string>
#include <list>
#include <filesystem>
#include <map>
#include <memory>
// gtest
#include <gtest/gtest.h>
// KnowRob
#include "knowrob/ThreadPool.h"
#include "knowrob/terms/Term.h"
#include "PrologEngine.h"
#include "knowrob/queries/AnswerYes.h"
#include "PrologBackend.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/reasoner/GoalDrivenReasoner.h"

namespace knowrob {
	/**
	 * A Prolog reasoner that answers queries using SWI Prolog.
	 */
	class PrologReasoner : public GoalDrivenReasoner {
	public:
		PrologReasoner();

		~PrologReasoner() override;

		/**
		 * Cannot be copy-assigned.
		 */
		PrologReasoner(const PrologReasoner &) = delete;

		/**
		 *
		 * @param key
		 * @param valueString
		 * @return
		 */
		bool setReasonerSetting(const TermPtr &key, const TermPtr &valueString);

		/**
		 * Consults a Prolog file, i.e. loads facts and rules and executed
		 * directives in the file.
		 * May throw an exception if there is no valid Prolog file at the given path.
		 * @prologFile the local path to the file.
		 * @return true on success
		 */
		bool consult(const std::filesystem::path &uri, const char *module = {}, bool doTransformQuery = true);

		/**
		 * @param rdfFile a rdf-xml encoded file.
		 */
		bool load_rdf_xml(const std::filesystem::path &rdfFile);

		/**
		 */
		virtual std::string_view callFunctor();

		PrologTerm transformGoal(const PrologTerm &goal);

		static std::shared_ptr<NamedReasoner> getDefinedReasoner(
				const term_t &t_reasonerManager, const term_t &t_reasonerModule);

		std::list<TermPtr> runTests(const std::string &target);

		// Override Reasoner
		bool initializeReasoner(const PropertyTree &cfg) override;

		// Override Reasoner
		void setDataBackend(const DataBackendPtr &backend) override;

		// Override Reasoner
		TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override;

	protected:
		static bool isKnowRobInitialized_;
		std::shared_ptr<PrologBackend> knowledgeGraph_;

		virtual bool initializeGlobalPackages() { return true; }

		virtual bool initializeDefaultPackages() { return true; }

		bool loadDataSourceWithUnknownFormat(const DataSourcePtr &dataFile) override {
			return consult(dataFile->uri());
		};

		AnswerYesPtr yes(const FramedTriplePatternPtr &literal, const PrologTerm &rdfGoal, const PrologTerm &frameTerm);

		AnswerNoPtr no(const FramedTriplePatternPtr &literal);

		static bool putQueryFrame(PrologTerm &frameTerm, const GraphSelector &frame);

		static std::shared_ptr<GraphSelector> createAnswerFrame(const PrologTerm &plTerm);

		PrologTerm getReasonerQuery(const PrologTerm &goal);
	};
}

#endif //KNOWROB_PROLOG_REASONER_H_
