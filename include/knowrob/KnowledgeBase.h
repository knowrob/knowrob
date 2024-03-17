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
#include "knowrob/db/OntologyFile.h"
#include "knowrob/db/QueryableBackend.h"
#include "knowrob/db/DefinedBackend.h"
#include "knowrob/db/BackendManager.h"
#include "knowrob/triples/GraphPathQuery.h"
#include "knowrob/db/BackendInterface.h"

namespace knowrob {
	// forward declaration
	class ReasonerManager;

	/**
	 * The main interface to the knowledge base system implementing
	 * its 'tell' and 'ask' interface.
	 */
	class KnowledgeBase {
	public:
		/**
		 * @param config a property tree used to configure this.
		 */
		explicit KnowledgeBase(const boost::property_tree::ptree &config);

		/**
		 * @param configFile path to file that encodes a boost property tree used to configure the KB.
		 */
		explicit KnowledgeBase(const std::string_view &configFile);

		KnowledgeBase();

		~KnowledgeBase();

		void init();

		void loadCommon();

		/**
		 * Load a data source into the knowledge base.
		 * This will potentially load the data source into multiple backends
		 * depending on which data formats are supported by the backends.
		 * @param source the data source to load
		 * @return true if the data source was loaded successfully
		 */
		bool loadDataSource(const DataSourcePtr &source);

		/**
		 * @return the vocabulary of this knowledge base, i.e. all known properties and classes
		 */
		auto &vocabulary() const { return vocabulary_; }

		/**
		 * @param property a property IRI
		 * @return true if the property is materialized in the EDB
		 */
		bool isMaterializedInEDB(std::string_view property) const;

		/**
		 * @return import hierarchy of named graphs
		 */
		auto &importHierarchy() const { return importHierarchy_; }

		/**
		 * Evaluate a query represented as a vector of literals.
		 * The call is non-blocking and returns a stream of answers.
		 * @param literals a vector of literals
		 * @param label an optional modalFrame label
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const GraphPathQueryPtr &graphQuery);

		/**
		 * Evaluate a query represented as a Literal.
		 * The call is non-blocking and returns a stream of answers.
		 * @param query a literal
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const FirstOrderLiteralPtr &query, const QueryContextPtr &ctx);

		/**
		 * Evaluate a query represented as a Formula.
		 * The call is non-blocking and returns a stream of answers.
		 * @param query a formula
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const FormulaPtr &query, const QueryContextPtr &ctx);

		auto &reasonerManager() const { return reasonerManager_; }

		auto &backendManager() const { return backendManager_; }

		bool insertOne(const FramedTriple &triple);

		bool insertAll(const semweb::TripleContainerPtr &triples);

		bool insertAll(const std::vector<FramedTriplePtr> &triples);

		bool removeOne(const FramedTriple &triple);

		bool removeAll(const semweb::TripleContainerPtr &triples);

		bool removeAll(const std::vector<FramedTriplePtr> &triples);

		bool removeAllWithOrigin(std::string_view origin);

		auto &edb() const { return edb_; }

		QueryableBackendPtr getBackendForQuery() const;

	protected:
		std::shared_ptr<BackendInterface> edb_;
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<BackendManager> backendManager_;
		std::shared_ptr<Vocabulary> vocabulary_;
		std::shared_ptr<ImportHierarchy> importHierarchy_;
		bool isInitialized_;

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
			explicit EDBComparator(VocabularyPtr vocabulary)
					: vocabulary_(std::move(vocabulary)) {}

			bool operator()(const FramedTriplePatternPtr &a, const FramedTriplePatternPtr &b) const;

			VocabularyPtr vocabulary_;
		};

		struct IDBComparator {
			explicit IDBComparator(VocabularyPtr vocabulary)
					: vocabulary_(std::move(vocabulary)) {}

			bool operator()(const RDFComputablePtr &a, const RDFComputablePtr &b) const;

			VocabularyPtr vocabulary_;
		};

		void configure(const boost::property_tree::ptree &config);

		static void configurePrefixes(const boost::property_tree::ptree &config);

		void configureDataSources(const boost::property_tree::ptree &config);

		void configureBackends(const boost::property_tree::ptree &config);

		void configureReasoner(const boost::property_tree::ptree &config);

		void initVocabulary();

		void initBackends();

		void synchronizeBackends();

		std::shared_ptr<DefinedBackend> findSourceBackend(const FramedTriple &triple);

		static DataSourcePtr createDataSource(const boost::property_tree::ptree &subtree);

		void startReasoner();

		void stopReasoner();

		std::vector<std::shared_ptr<DefinedBackend>>
		prepareLoad(std::string_view origin, std::string_view newVersion) const;

		void
		finishLoad(const std::shared_ptr<OntologySource> &source, std::string_view origin, std::string_view newVersion);

		bool loadNonOntologySource(const DataSourcePtr &source) const;

		bool loadOntologyFile(const std::shared_ptr<OntologyFile> &source, bool followImports = true);

		bool loadSPARQLDataSource(const std::shared_ptr<DataSource> &source);

		static DataSourceType getDataSourceType(const std::string &format, const boost::optional<std::string> &language,
												const boost::optional<std::string> &type);

		std::optional<std::string> getVersionOfOrigin(const std::shared_ptr<DefinedBackend> &definedBackend,
													  std::string_view origin) const;

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
