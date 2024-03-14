/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_TRANSACTION_H
#define KNOWROB_DATA_TRANSACTION_H

#include "memory"
#include "knowrob/ThreadPool.h"
#include "knowrob/db/DataBackend.h"
#include "knowrob/triples/TripleContainer.h"
#include "DefinedBackend.h"

namespace knowrob {
	/**
	 * A data transaction of a set of triples that are either inserted or removed from a data backend.
	 */
	class DataTransaction : public ThreadPool::Runner {
	public:
		DataTransaction(const std::shared_ptr<IDataBackend> &backend, std::string_view backendName,
						const semweb::TripleContainerPtr &triples);

	protected:
		std::shared_ptr<IDataBackend> backend_;
		std::string_view backendName_;
		semweb::TripleContainerPtr triples_;
	};

	/**
	 * A data insertion transaction.
	 */
	class DataInsertion : public DataTransaction {
	public:
		DataInsertion(const std::shared_ptr<IDataBackend> &backend, std::string_view backendName,
					  const semweb::TripleContainerPtr &triples);

		void run() override;
	};

	/**
	 * A data removal transaction.
	 */
	class DataRemoval : public DataTransaction {
	public:
		DataRemoval(const std::shared_ptr<IDataBackend> &backend, std::string_view backendName,
					const semweb::TripleContainerPtr &triples);

		void run() override;
	};
}

#endif //KNOWROB_DATA_TRANSACTION_H
