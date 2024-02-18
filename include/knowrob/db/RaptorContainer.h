/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_RAPTOR_CONTAINER_H
#define KNOWROB_RAPTOR_CONTAINER_H

#include <raptor.h>
#include <librdf.h>
#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/semweb/StatementData.h"

namespace knowrob {
	/**
	 * A batch of triples loaded with raptor.
	 */
	class RaptorContainer : public semweb::MutableTripleContainer {
	public:
		/**
		 * @param size the maximum number of triples to be stored.
		 * @param origin the origin of the triples.
		 */
		RaptorContainer(uint32_t size, std::string_view origin);

		/**
		 * @param size the maximum number of triples to be stored.
		 */
		explicit RaptorContainer(uint32_t size);

		~RaptorContainer();

		/**
		 * Add a triple to the batch.
		 * @param s a raptor term.
		 * @param p a raptor term.
		 * @param o a raptor term.
		 * @return the added statement.
		 */
		StatementData *add(raptor_term *s, raptor_term *p, raptor_term *o, librdf_node *context = nullptr);

		/**
		 * Add a triple to the batch.
		 * @param statement a raptor statement.
		 * @return the added statement.
		 */
		StatementData *add(raptor_statement *statement, librdf_node *context = nullptr);

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

		// override TripleContainer
		const std::vector<StatementData> &asImmutableVector() const override { return mappedData_; }

		// override TripleContainer
		std::vector<StatementData> &asMutableVector() override { return mappedData_; }

	protected:
		struct mapped_statement {
			raptor_term *s;
			raptor_term *p;
			raptor_term *o;
		};
		std::vector<mapped_statement> raptorData_;
		std::vector<StatementData> mappedData_;
		uint32_t actualSize_;
		std::optional<std::string_view> origin_;
	};

} // knowrob

#endif //KNOWROB_RAPTOR_CONTAINER_H
