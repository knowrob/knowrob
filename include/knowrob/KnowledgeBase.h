/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWLEDGE_BASE_H
#define KNOWROB_KNOWLEDGE_BASE_H

#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <utility>
#include "knowrob/queries/QueryContext.h"
#include "knowrob/storage/QueryableStorage.h"
#include "knowrob/storage/StorageManager.h"
#include "knowrob/storage/StorageInterface.h"
#include "knowrob/semweb/GraphPathQuery.h"
#include "knowrob/semweb/OntologySource.h"
#include "knowrob/formulas/SimpleConjunction.h"
#include "knowrob/queries/ConjunctiveQuery.h"
#include "knowrob/storage/Observer.h"
#include "knowrob/storage/ObserverManager.h"

namespace knowrob {
	// forward declaration
	class ReasonerManager;

	/**
	 * The main interface to the knowledge base system implementing
	 * its 'tell' and 'ask' interface.
	 * Note that it is perfectly fine to have multiple KnowledgeBase instances in one application.
	 */
	class KnowledgeBase : public std::enable_shared_from_this<KnowledgeBase> {
	public:
		/**
		 * Create a new KnowledgeBase instance.
		 * @param config a property tree used to configure this.
		 */
		static std::shared_ptr<KnowledgeBase> create(const boost::property_tree::ptree &config);

		/**
		 * Create a new KnowledgeBase instance.
		 * @param config a JSON string used to configure this or the path to a JSON file.
		 */
		static std::shared_ptr<KnowledgeBase> create(std::string_view config);

		/**
		 * Create a new KnowledgeBase instance.
		 */
		static std::shared_ptr<KnowledgeBase> create();

		~KnowledgeBase();

		void init();

		void loadCommon();

		/**
		 * Load a data source into the knowledge base, possibly loading it into multiple backends.
		 * @param source the data source to load
		 * @return true if the data source was loaded successfully
		 */
		bool loadDataSource(const DataSourcePtr &source);

		/**
		 * @return the vocabulary of this knowledge base, i.e. all known properties and classes
		 */
		auto &vocabulary() const { return vocabulary_; }

		/**
		 * @return the storage interface of this knowledge base
		 */
		auto &edb() const { return edb_; }

		/**
		 * @return the reasoner manager of this knowledge base
		 */
		auto &reasonerManager() const { return reasonerManager_; }

		/**
		 * @return the storage manager of this knowledge base
		 */
		auto &backendManager() const { return backendManager_; }

		/**
		 * @return the default graph of this knowledge base
		 */
		QueryableBackendPtr getBackendForQuery() const;

		/**
		 * Evaluate a query represented as a vector of literals.
		 * The call is non-blocking and returns a stream of answers.
		 * @param conjunctiveQuery the query
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const ConjunctiveQueryPtr &conjunctiveQuery);

		/**
		 * Evaluate a query represented as a Literal.
		 * The call is non-blocking and returns a stream of answers.
		 * @param query a literal
		 * @param ctx a query context
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const FirstOrderLiteralPtr &query, const QueryContextPtr &ctx);

		/**
		 * Evaluate a query represented as a Formula.
		 * The call is non-blocking and returns a stream of answers.
		 * @param query a formula
		 * @param ctx a query context
		 * @return a stream of query results
		 */
		TokenBufferPtr submitQuery(const FormulaPtr &query, const QueryContextPtr &ctx);

		/**
		 * Observe a query represented as a graph query.
		 * @param query a graph query
		 * @param callback a function that is called for each answer to the query
		 * @return an observer that can be used to cancel the query
		 */
		ObserverPtr observe(const GraphQueryPtr &query, const BindingsHandler &callback);

		/**
		 * Block until all observers have processed all queued data.
		 */
		void synchronizeObservers();

		/**
		 * Insert a single triple into the knowledge base.
		 * @param triple the triple to insert
		 * @return true if the triple was inserted successfully
		 */
		bool insertOne(const Triple &triple);

		/**
		 * Insert a collection of triples into the knowledge base.
		 * @param triples the triples to insert
		 * @return true if the triples were inserted successfully
		 */
		bool insertAll(const TripleContainerPtr &triples);

		/**
		 * Insert a collection of triples into the knowledge base.
		 * @param triples the triples to insert
		 * @return true if the triples were inserted successfully
		 */
		bool insertAll(const std::vector<TriplePtr> &triples);

		/**
		 * Remove a single triple from the knowledge base.
		 * @param triple the triple to remove
		 * @return true if the triple was removed successfully
		 */
		bool removeOne(const Triple &triple);

		/**
		 * Remove a collection of triples from the knowledge base.
		 * @param triples the triples to remove
		 * @return true if the triples were removed successfully
		 */
		bool removeAll(const TripleContainerPtr &triples);

		/**
		 * Remove a collection of triples from the knowledge base.
		 * @param triples the triples to remove
		 * @return true if the triples were removed successfully
		 */
		bool removeAll(const std::vector<TriplePtr> &triples);

		/**
		 * Remove all triples with a given origin from the knowledge base.
		 * @param origin the origin of the triples to remove
		 * @return true if the triples were removed successfully
		 */
		bool removeAllWithOrigin(std::string_view origin);

		/**
		 * Set the default graph for queries.
		 * @param origin the origin of the default graph
		 */
		void setDefaultGraph(std::string_view origin);

	protected:
		std::shared_ptr<StorageInterface> edb_;
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<StorageManager> backendManager_;
		std::shared_ptr<Vocabulary> vocabulary_;
		std::shared_ptr<ObserverManager> observerManager_;
		bool isInitialized_;

		explicit KnowledgeBase(const boost::property_tree::ptree &config);

		explicit KnowledgeBase(std::string_view config);

		explicit KnowledgeBase();

		void configure(const boost::property_tree::ptree &config);

		static void configurePrefixes(const boost::property_tree::ptree &config);

		void configureDataSources(const boost::property_tree::ptree &config);

		void configureBackends(const boost::property_tree::ptree &config);

		void configureReasoner(const boost::property_tree::ptree &config);

		void initVocabulary();

		void addToVocabulary(const TriplePtr &triple);

		void initBackends();

		void synchronizeBackends();

		std::shared_ptr<NamedBackend> findSourceBackend(const Triple &triple);

		void startReasoner();

		void stopReasoner();

		std::vector<std::shared_ptr<NamedBackend>>
		prepareLoad(std::string_view origin, std::string_view newVersion) const;

		void
		finishLoad(const std::shared_ptr<OntologySource> &source, std::string_view origin, std::string_view newVersion);

		bool loadNonOntologySource(const DataSourcePtr &source) const;

		bool loadOntologySource(const std::shared_ptr<OntologySource> &source);

		std::optional<std::string> getVersionOfOrigin(const std::shared_ptr<NamedBackend> &definedBackend,
													  std::string_view origin) const;
	};

	using KnowledgeBasePtr = std::shared_ptr<KnowledgeBase>;
}

#endif //KNOWROB_KNOWLEDGE_BASE_H
