/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWLEDGE_BASE_H
#define KNOWROB_KNOWLEDGE_BASE_H

#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <utility>
#include "ThreadPool.h"
#include "knowrob/formulas/DependencyGraph.h"
#include "knowrob/queries/QueryPipeline.h"
#include "knowrob/queries/QueryContext.h"
#include "knowrob/semweb/RDFComputable.h"
#include "knowrob/reasoner/DefinedReasoner.h"

namespace knowrob {
	enum QueryFlag {
		QUERY_FLAG_ALL_SOLUTIONS = 1 << 0,
		QUERY_FLAG_ONE_SOLUTION = 1 << 1,
		QUERY_FLAG_PERSIST_SOLUTIONS = 1 << 2,
		QUERY_FLAG_UNIQUE_SOLUTIONS = 1 << 3,
		//QUERY_FLAG_ORDER_PRESERVING = 1 << 4,
	};

	// forward declaration
	class ReasonerManager;

	// forward declaration
	class BackendManager;

	/**
	 * The main interface to the knowledge base system implementing
	 * its 'tell' and 'ask' interface.
	 */
	class KnowledgeBase : public IDataBackend {
	public:
		/**
		 * @param config a property tree used to configure this.
		 */
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

		/**
		 * @param configFile path to file that encodes a boost property tree used to configure the KB.
		 */
		explicit KnowledgeBase(const std::string_view &configFile);

		~KnowledgeBase();

		/**
		 * @return the set of reasoner of this KB.
		 */
		const std::map<std::string, std::shared_ptr<DefinedReasoner>> &reasonerPool() const;

		/**
		 * @return the central knowledge graph
		 */
		auto centralKG() const { return centralKG_; }

		/**
		 * @return the vocabulary of this knowledge base, i.e. all known properties and classes
		 */
		auto vocabulary() const { return centralKG()->vocabulary(); }

		/**
		 * @param property a property IRI
		 * @return true if the property is materialized in the EDB
		 */
		bool isMaterializedInEDB(std::string_view property) const;

		/**
		 * @return import hierarchy of named graphs
		 */
		auto importHierarchy() const { return centralKG()->importHierarchy(); }

		/**
		 * Evaluate a query represented as a vector of literals.
		 * The call is non-blocking and returns a stream of answers.
		 * @param literals a vector of literals
		 * @param label an optional modalFrame label
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const ConjunctiveQueryPtr &graphQuery);

		/**
		 * Evaluate a query represented as a Literal.
		 * The call is non-blocking and returns a stream of answers.
		 * @param query a literal
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const LiteralPtr &query, const QueryContextPtr &ctx);

		/**
		 * Evaluate a query represented as a Formula.
		 * The call is non-blocking and returns a stream of answers.
		 * @param query a formula
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const FormulaPtr &query, const QueryContextPtr &ctx);

		//auto &reasonerManager() const { return reasonerManager_; }

		// override IDataBackend
		bool insertOne(const StatementData &triple) override;

		// override IDataBackend
		bool insertAll(const std::vector<StatementData> &triples) override;

		// override IDataBackend
		bool removeOne(const StatementData &triple) override;

		// override IDataBackend
		bool removeAll(const std::vector<StatementData> &triples) override;

		// override IDataBackend
		int removeMatching(const RDFLiteral &query, bool doMatchMany) override;

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<BackendManager> backendManager_;
		std::shared_ptr<KnowledgeGraph> centralKG_;

		// used to sort dependency nodes in a priority queue.
		// the nodes are considered to be dependent on each other through free variables.
		// the priority value is used to determine which nodes should be evaluated first.
		struct DependencyNodeComparator {
			bool operator()(const DependencyNodePtr &a, const DependencyNodePtr &b) const;
		};

		struct DependencyNodeQueue {
			const DependencyNodePtr node_;
			std::priority_queue<DependencyNodePtr, std::vector<DependencyNodePtr>, DependencyNodeComparator> neighbors_;

			explicit DependencyNodeQueue(const DependencyNodePtr &node);
		};

		// compares literals
		struct EDBComparator {
			explicit EDBComparator(semweb::VocabularyPtr vocabulary)
					: vocabulary_(std::move(vocabulary)) {}

			bool operator()(const RDFLiteralPtr &a, const RDFLiteralPtr &b) const;

			semweb::VocabularyPtr vocabulary_;
		};

		struct IDBComparator {
			explicit IDBComparator(semweb::VocabularyPtr vocabulary)
					: vocabulary_(std::move(vocabulary)) {}

			bool operator()(const RDFComputablePtr &a, const RDFComputablePtr &b) const;

			semweb::VocabularyPtr vocabulary_;
		};

		void loadConfiguration(const boost::property_tree::ptree &config);

		void startReasoner();

		void stopReasoner();

		DataBackendPtr findSourceBackend(const StatementData &triple);

		std::vector<RDFComputablePtr> createComputationSequence(
				const std::list<DependencyNodePtr> &dependencyGroup) const;

		void createComputationPipeline(
				const std::shared_ptr<QueryPipeline> &pipeline,
				const std::vector<RDFComputablePtr> &computableLiterals,
				const std::shared_ptr<TokenBroadcaster> &pipelineInput,
				const std::shared_ptr<TokenBroadcaster> &pipelineOutput,
				const QueryContextPtr &ctx) const;
	};

	using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGE_BASE_H
