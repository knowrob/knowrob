//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_DATA_BACKEND_H
#define KNOWROB_DATA_BACKEND_H

#include <boost/property_tree/ptree.hpp>
#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/reasoner/ReasonerConfig.h"

namespace knowrob {
	/**
	 * A data backend is a component that stores extensional data.
	 * The knowledge base employs a central data backend that is used
	 * to store all extensional data.
	 * However, reasoners may use their own data backend to store
	 * extensional data that is used to evaluate axioms and rules.
	 */
    class DataBackend {
    public:
        DataBackend() = default;

		virtual ~DataBackend()= default;

        /**
         * Initialize this backend from a property tree.
         * @param config a property tree.
         * @return true on success
         */
        virtual bool loadConfig(const ReasonerConfig &config) = 0;

        /**
         * Add an assertion to this KG.
         * @param tripleData data representing an atomic proposition.
         * @return true on success
         */
        virtual bool insertOne(const StatementData &tripleData) = 0;

        /**
         * Add assertions to this KG.
         * @param data data representing atomic propositions.
         * @return true on success
         */
        virtual bool insertAll(const std::vector<StatementData> &data) = 0;

        /**
         * Delete all matching statements from this KG.
         * @param literal an expression used to match statements in the KG.
         */
        virtual void removeAll(const RDFLiteral &literal) = 0;

        /**
         * Delete the first matching statement from this KG.
         * @param literal an expression used to match statements in the KG.
         */
        virtual void removeOne(const RDFLiteral &literal) = 0;
    };

    using DataBackendPtr = std::shared_ptr<DataBackend>;

} // knowrob

#endif //KNOWROB_DATA_BACKEND_H
