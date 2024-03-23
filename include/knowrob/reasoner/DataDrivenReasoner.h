/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_DRIVEN_REASONER_H
#define KNOWROB_DATA_DRIVEN_REASONER_H

#include "Reasoner.h"

namespace knowrob {
	using InferredTripleContainer = std::shared_ptr<std::vector<std::shared_ptr<FramedTriple>>>;

	/**
	 * A reasoner that is driven by data, i.e., it starts with the data and infers
	 * additional knowledge e.g. by applying rules to the data.
	 * This is in contrast to a goal-driven reasoner, which is driven by queries.
	 * The data is taken from a data backend, which is associated with the reasoner
	 * before the reasoner is started.
	 */
	class DataDrivenReasoner : public Reasoner {
	public:
		DataDrivenReasoner() : Reasoner() {}

		/**
		 * Start the reasoner.
		 */
		virtual void start() = 0;

		/**
		 * Stop the reasoner, and destroy all resources.
		 * No further queries can be submitted after this function has been called.
		 */
		virtual void stop() = 0;

		/**
		 * Create a vector of triples that can be used to insert or remove data from the reasoner's data backend.
		 * @param count the number of triples to create.
		 * @return a vector of triples.
		 */
		InferredTripleContainer createTriples(uint32_t count) const;

		/**
		 * Set the inferred triples of this reasoner.
		 * Calling this function will replace the current set of inferred triples, and will
		 * delete all triples from the KB that are not in the inferred set anymore, and add all triples
		 * to the KB that newly appear.
		 * @param triples a vector of inferred triples.
		 */
		void setInferredTriples(const InferredTripleContainer &triples);

		/**
		 * Add additional triples to the inferred set of this reasoner,
		 * and adding the inferred triples to the KB.
		 * @param triples a vector of inferred triples.
		 */
		void addInferredTriples(const InferredTripleContainer &triples) const;

		/**
		 * Remove triples from the inferred set of this reasoner,
		 * and remove the inferred triples from the KB.
		 * @param triples a vector of inferred triples.
		 */
		void removeInferredTriples(const InferredTripleContainer &triples) const;

	protected:
		struct InferredComparator {
			bool operator()(const std::shared_ptr<FramedTriple> &v0,
							const std::shared_ptr<FramedTriple> &v1) const;
		};

		using InferredeSet = std::set<std::shared_ptr<FramedTriple>, InferredComparator>;
		InferredeSet inferredTriples_;
	};

	using DataDrivenReasonerPtr = std::shared_ptr<DataDrivenReasoner>;
}

#endif //KNOWROB_DATA_DRIVEN_REASONER_H
