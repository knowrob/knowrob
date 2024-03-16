/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BACKEND_INTERFACE_H
#define KNOWROB_BACKEND_INTERFACE_H

#include "knowrob/db/QueryableBackend.h"
#include "BackendManager.h"
#include "BackendTransaction.h"

namespace knowrob {
	/**
	 * A high-level interface to the backend manager.
	 * It includes methods for querying and modifying the extensional database.
	 */
	class BackendInterface {
	public:
		/**
		 * The type of a transaction.
		 */
		enum TransactionType {
			Insert, Remove
		};
		/**
		 * Determines how backends are selected for a transaction.
		 */
		enum BackendSelection {
			Including, Excluding
		};

		explicit BackendInterface(const std::shared_ptr<BackendManager> &backendManager)
				: backendManager_(backendManager) {}

		/**
		 * @return the vocabulary.
		 */
		auto &vocabulary() const { return backendManager_->vocabulary(); }

		/**
		 * @return the import hierarchy.
		 */
		auto &importHierarchy() const { return backendManager_->importHierarchy(); }

		/**
		 * @return the backend manager.
		 */
		auto &backendManager() const { return backendManager_; }

		/**
		 * Creates a new transaction.
		 * @param queryable a backend used to perform any queries needed to complete the transaction.
		 * @param type the type of the transaction.
		 * @param mode determines how backends are selected for the transaction.
		 * @param backends the backends to include or exclude from the transaction.
		 * @return the transaction.
		 */
		std::shared_ptr<transaction::Transaction> createTransaction(
				const QueryableBackendPtr &queryable,
				TransactionType type,
				BackendSelection mode = Excluding,
				const std::vector<std::shared_ptr<DefinedBackend>> &transactionBackends = {});

		/**
		 * Removes all triples with a given origin from all backends.
		 * @param origin the origin of the triples to remove.
		 * @return true if the triples were removed from all backends, false otherwise.
		 */
		bool removeAllWithOrigin(std::string_view origin);

		/**
		 * Inserts a triple into the extensional database and merges it with existing ones
		 * if possible.
		 * @param backend the backend to modify.
		 * @param triple the triple to insert.
		 * @return true if the triple was inserted, false otherwise.
		 */
		bool mergeInsert(const QueryableBackendPtr &backend, const FramedTriple &triple);

		/**
		 * Checks if a triple is contained in the extensional database.
		 * @param backend the backend to query.
		 * @param triple the triple to check.
		 * @return true if the triple is contained in the extensional database, false otherwise.
		 */
		bool contains(const QueryableBackendPtr &backend, const FramedTriple &triple) const;

		/**
		 * Executes a visitor on all triples in the extensional database.
		 * @param backend the backend to query.
		 * @param visitor the visitor to execute.
		 */
		static void foreach(const QueryableBackendPtr &backend, const semweb::TripleVisitor &visitor);

		/**
		 * Executes a visitor on all triples in the extensional database.
		 * @param backend the backend to query.
		 * @param callback the visitor to execute.
		 */
		static void batch(const QueryableBackendPtr &backend, const semweb::TripleHandler &callback);

		/**
		 * Evaluates a query on the extensional database and executes a visitor on the results
		 * which are returned in the form of triples.
		 * @param backend the backend to query.
		 * @param query the query to evaluate.
		 * @param visitor the visitor to execute.
		 */
		void match(const QueryableBackendPtr &backend, const FramedTriplePattern &query,
				   const semweb::TripleVisitor &visitor) const;

		/**
		 * Evaluates a query on the extensional database and executes a visitor on the results
		 * which are returned in as bindings.
		 * @param backend the backend to query.
		 * @param q the query to evaluate.
		 * @param callback the visitor to execute.
		 */
		void query(const QueryableBackendPtr &backend, const GraphQueryPtr &q, const BindingsHandler &callback) const;

		/**
		 * Evaluates a query on the extensional database and fills a token buffer with the results
		 * in a separate thread.
		 * @param backend the backend to query.
		 * @param query the query to evaluate.
		 * @return the token buffer with the results.
		 */
		TokenBufferPtr getAnswerCursor(const QueryableBackendPtr &backend, const GraphPathQueryPtr &query);

	protected:
		std::shared_ptr<BackendManager> backendManager_;

		void pushIntoCursor(const QueryableBackendPtr &backend, const GraphPathQueryPtr &query,
							const TokenBufferPtr &resultStream) const;
	};

	using TransactionCtrlPtr = std::shared_ptr<BackendInterface>;

} // knowrob

#endif //KNOWROB_BACKEND_INTERFACE_H
