//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_DATA_BACKEND_H
#define KNOWROB_DATA_BACKEND_H

#include <boost/property_tree/ptree.hpp>
#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/reasoner/ReasonerConfig.h"
#include "knowrob/ThreadPool.h"
#include "DataSourceHandler.h"

namespace knowrob {
	/**
	 * Interface for data backends that can store triple data.
	 */
	class IDataBackend {
	public:
		/**
		 * Add an assertion to this backend.
		 * @param triple a triple.
		 * @return true on success
		 */
		virtual bool insertOne(const StatementData &triple) = 0;

		/**
		 * Add assertions to this backend.
		 * @param triples a set of triples.
		 * @return true on success
		 */
		virtual bool insertAll(const std::vector<StatementData> &triples) = 0;

		/**
		 * Delete the first matching statement from this backend.
		 * @param triple a triple.
		 */
		virtual bool removeOne(const StatementData &triple) = 0;

		/**
		 * Delete all matching statements from this backend.
		 * @param triples a set of triples.
		 */
		virtual bool removeAll(const std::vector<StatementData> &triples) = 0;

		/**
		 * Delete all matching statements from this backend.
		 * @param query an expression used to match triples in the backend.
		 * @param doMatchMany if true, all matching triples are deleted, otherwise only the first one.
		 * @return the number of deleted triples.
		 */
		virtual int removeMatching(const RDFLiteral &query, bool doMatchMany) = 0;
	};

	/**
	 * A data backend is a component that stores extensional data.
	 * The knowledge base employs a central data backend that is used
	 * to store all extensional data.
	 * However, reasoners may use their own data backend to store
	 * extensional data that is used to evaluate axioms and rules.
	 */
	class DataBackend : public IDataBackend, public DataSourceHandler {
	public:
		DataBackend() = default;

		virtual ~DataBackend() = default;

		/**
		 * Initialize this backend from a property tree.
		 * @param config a property tree.
		 * @return true on success
		 */
		virtual bool loadConfig(const ReasonerConfig &config) = 0;
	};

	using DataBackendPtr = std::shared_ptr<DataBackend>;

} // knowrob

#endif //KNOWROB_DATA_BACKEND_H
