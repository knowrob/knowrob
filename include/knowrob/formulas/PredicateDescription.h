//
// Created by daniel on 27.01.24.
//

#ifndef KNOWROB_PREDICATE_DESCRIPTION_H
#define KNOWROB_PREDICATE_DESCRIPTION_H

#include "memory"
#include "PredicateIndicator.h"

namespace knowrob {
	/**
	 * The type of a predicate.
	 */
	enum class PredicateType : uint8_t {
		/**
		 * The predicate is a built-in predicate.
		 */
		BUILT_IN = 0,
		/**
		 * The predicate is a relation that only appears in the EDB.
		 */
		EDB_RELATION,
		/**
		 * The predicate is a relation that may appear in the IDB.
		 */
		IDB_RELATION
	};

	/**
	 * Read predicate type from term.
	 * @param term a term.
	 * @return the predicate type encoded by term.
	 */
	PredicateType predicateTypeFromTerm(const TermPtr &term);

	/**
	 * The materialization strategy of a predicate.
	 * This determines whether and when an IDB predicate is stored in the memory or not.
	 */
	enum class MaterializationStrategy : uint8_t {
		/**
		 * The predicate should not be materialized in the memory.
		 * Thus, the reasoner needs to be always queried to retrieve facts of the predicate.
		 */
		NEVER = 0,
		/**
		 * The predicate is materialized by the reasoner.
		 * So at any point in time, the EDB contains all facts of the predicate.
		 * Thus, the reasoner does not need to be queried to retrieve facts of the predicate
		 * and more efficient queries can be used.
		 */
		ALWAYS,
		/**
		 * The predicate is materialized, but only if it is used in a query.
		 * The knowledge base system keeps track of which predicates are used in queries,
		 * and materializes them if they are used for the arguments given in the query.
		 */
		ON_DEMAND
	};

	/**
	 * The description of a defined predicate.
	 */
	class PredicateDescription {
	public:
		/**
		 * @param indicator the indicator of the predicate.
		 * @param type the type of the predicate.
		 */
		PredicateDescription(
				const std::shared_ptr<PredicateIndicator> &indicator,
				PredicateType type,
				MaterializationStrategy materializationStrategy = MaterializationStrategy::ON_DEMAND);

		/**
		 * @return the indicator of the predicate.
		 */
		const std::shared_ptr<PredicateIndicator>& indicator() const { return indicator_; }

		/**
		 * @return the type of the predicate.
		 */
		PredicateType type() const { return type_; }

		/**
		 * @return the materialization strategy of the predicate.
		 */
		MaterializationStrategy materializationStrategy() const { return materializationStrategy_; }

	protected:
		std::shared_ptr<PredicateIndicator> indicator_;
		PredicateType type_;
		MaterializationStrategy materializationStrategy_;
	};

    using PredicateDescriptionPtr = std::shared_ptr<PredicateDescription>;
}

#endif //KNOWROB_PREDICATE_DESCRIPTION_H
