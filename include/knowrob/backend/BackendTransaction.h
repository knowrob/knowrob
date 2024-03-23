/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_TRANSACTION_H
#define KNOWROB_DATA_TRANSACTION_H

#include "memory"
#include "knowrob/ThreadPool.h"
#include "knowrob/backend/Backend.h"
#include "knowrob/triples/TripleContainer.h"
#include "QueryableBackend.h"
#include "knowrob/reification/ReificationContainer.h"

namespace knowrob::transaction {
	/**
	 * Baseclass for transactions.
	 * A set of backends can be added to a transaction. When a transaction is committed,
	 * the triple is committed to all backends. If a backend does not support the context
	 * of the triple, the triple is reified and the reified triples are committed to the backend.
	 */
	class Transaction {
	public:
		Transaction(const std::shared_ptr<QueryableBackend> &queryable,
					const std::shared_ptr<Vocabulary> &vocabulary,
					bool isRemoval)
				: queryable_(queryable),
				  vocabulary_(vocabulary),
				  isRemoval_(isRemoval) {}

		/**
		 * Adds a backend to the transaction.
		 * @param backend the backend to add.
		 */
		void addBackend(const std::shared_ptr<NamedBackend> &backend) { backends_.push_back(backend); }

		/**
		 * Commits a triple to all backends.
		 * @param triple the triple to commit.
		 * @return true if the triple was committed to all backends, false otherwise.
		 */
		bool commit(const FramedTriple &triple);

		/**
		 * Commits a triple to all backends.
		 * @param triple the triple to commit.
		 * @param reifiedName the reified name of the triple.
		 * @return true if the triple was committed to all backends, false otherwise.
		 */
		bool commit(const FramedTriple &triple, const IRIAtomPtr &reifiedName);

		/**
		 * Commits a set of triples to all backends.
		 * @param triples the triples to commit.
		 * @return true if the triples were committed to all backends, false otherwise.
		 */
		bool commit(const TripleContainerPtr &triples);

		/**
		 * Commits a set of triples to all backends.
		 * @param triples the triples to commit.
		 * @param reifiedNames the reified names of the triples.
		 * @return true if the triples were committed to all backends, false otherwise.
		 */
		bool commit(const TripleContainerPtr &triples, const ReifiedNames &reifiedNames);

	protected:
		std::shared_ptr<Vocabulary> vocabulary_;
		std::shared_ptr<QueryableBackend> queryable_;
		std::vector<std::shared_ptr<NamedBackend>> backends_;
		bool isRemoval_;

		virtual bool doCommit(const FramedTriple &triple, const DataBackendPtr &backend) = 0;

		virtual bool doCommit(const TripleContainerPtr &triples, const DataBackendPtr &backend) = 0;

		virtual void updateVocabulary(const FramedTriple &triple) = 0;

		static std::shared_ptr<ThreadPool::Runner> createTripleWorker(
				const TripleContainerPtr &triples,
				const std::function<void(const FramedTriplePtr &)> &fn);

		IRIAtomPtr queryReifiedName(const FramedTriple &triple);
	};

	/**
	 * A data insertion transaction.
	 */
	class Insert : public Transaction {
	public:
		Insert(const std::shared_ptr<QueryableBackend> &queryable, const std::shared_ptr<Vocabulary> &vocabulary)
				: Transaction(queryable, vocabulary, false) {}

	protected:
		bool doCommit(const FramedTriple &triple, const DataBackendPtr &backend) override;

		bool doCommit(const TripleContainerPtr &triples, const DataBackendPtr &backend) override;

		void updateVocabulary(const FramedTriple &triple) override;
	};

	/**
	 * A data removal transaction.
	 */
	class Remove : public Transaction {
	public:
		Remove(const std::shared_ptr<QueryableBackend> &queryable, const std::shared_ptr<Vocabulary> &vocabulary)
				: Transaction(queryable, vocabulary, true) {}

	protected:
		bool doCommit(const FramedTriple &triple, const DataBackendPtr &backend) override;

		bool doCommit(const TripleContainerPtr &triples, const DataBackendPtr &backend) override;

		void updateVocabulary(const FramedTriple &triple) override;
	};
}

#endif //KNOWROB_DATA_TRANSACTION_H
