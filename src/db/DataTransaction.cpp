/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/DataTransaction.h"

using namespace knowrob;

DataTransaction::DataTransaction(const std::shared_ptr<IDataBackend> &backend, std::string_view backendName,
								 const semweb::TripleContainerPtr &triples)
		: backend_(backend), backendName_(backendName), triples_(triples) {
}

DataInsertion::DataInsertion(const std::shared_ptr<IDataBackend> &backend, std::string_view backendName,
							 const semweb::TripleContainerPtr &triples)
		: DataTransaction(backend, backendName, triples) {
}

void DataInsertion::run() {
	if (!backend_->insertAll(triples_)) {
		KB_WARN("assertion of triple into backend '{}' failed!", backendName_);
	}
}

DataRemoval::DataRemoval(const std::shared_ptr<IDataBackend> &backend, std::string_view backendName,
						 const semweb::TripleContainerPtr &triples)
		: DataTransaction(backend, backendName, triples) {
}

void DataRemoval::run() {
	if (!backend_->removeAll(triples_)) {
		KB_WARN("removal of triple from backend '{}' failed!", backendName_);
	}
}
