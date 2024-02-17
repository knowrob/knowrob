/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TRIPLE_BATCH_H
#define KNOWROB_TRIPLE_BATCH_H

#include <raptor.h>
#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/semweb/StatementData.h"

namespace knowrob {
	/**
	 * A batch of triples loaded with raptor.
	 */
	class RaptorContainer : public semweb::TripleContainer {
	public:
		/**
		 * @param size the maximum number of triples to be stored.
		 * @param origin the origin of the triples.
		 */
		RaptorContainer(uint32_t size, std::string_view origin);

		~RaptorContainer();

		/**
		 * Add a triple to the batch.
		 * @param s a raptor term.
		 * @param p a raptor term.
		 * @param o a raptor term.
		 * @return the added statement.
		 */
		StatementData *add(raptor_term *s, raptor_term *p, raptor_term *o);

		/**
		 * Add a triple to the batch.
		 * @param statement a raptor statement.
		 * @return the added statement.
		 */
		StatementData *add(raptor_statement *statement);

		/**
		 * Reset the batch to be empty.
		 */
		void reset();

		/**
		 * Rollback the last added statement.
		 */
		void rollbackLast();

		/**
		 * Shrink the batch to the actual size.
		 */
		void shrink();

		/**
		 * @return the actual size of the batch.
		 */
		auto size() const { return actualSize_; }

		/**
		 * @return the origin for all triples in this batch.
		 */
		auto origin() const { return origin_; }

		// override TripleContainer
		const std::vector<StatementData> &asVector() const override { return mappedData_; }

	protected:
		struct mapped_statement {
			raptor_term *s;
			raptor_term *p;
			raptor_term *o;
		};
		std::vector<mapped_statement> raptorData_;
		std::vector<StatementData> mappedData_;
		uint32_t actualSize_;
		std::string_view origin_;
	};

} // knowrob

#endif //KNOWROB_TRIPLE_BATCH_H
