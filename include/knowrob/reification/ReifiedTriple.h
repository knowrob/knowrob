/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REIFIED_TRIPLE_H
#define KNOWROB_REIFIED_TRIPLE_H

#include "vector"
#include "knowrob/triples/FramedTriple.h"
#include "knowrob/semweb/Vocabulary.h"

namespace knowrob {
	/**
	 * A collection of triples that creates a reified view on another triple.
	 * Note that triple data is not copy, so caller should ensure that the
	 * original triple is not deleted while the reified view is used.
	 */
	class ReifiedTriple {
	public:
		/**
		 * Create a reified view on a triple.
		 * @param triple the triple to reify.
		 * @param vocabulary the vocabulary to use for reification.
		 */
		explicit ReifiedTriple(const FramedTriple &triple, const semweb::VocabularyPtr &vocabulary);

		/**
		 * @return begin iterator over the reified triples.
		 */
		auto begin() { return reified_.begin(); }

		/**
		 * @return end iterator over the reified triples.
		 */
		auto end() { return reified_.end(); }

		/**
		 * @return begin iterator over the reified triples.
		 */
		auto begin() const { return reified_.begin(); }

		/**
		 * @return end iterator over the reified triples.
		 */
		auto end() const { return reified_.end(); }

		/**
		 * Check if a triple belongs to the reified representation of another triple.
		 * @param triple the triple to check.
		 * @param vocabulary a vocabulary.
		 * @return true if the triple is part of a reification.
		 */
		static bool isPartOfReification(const FramedTriple &triple);

		/**
		 * Check if a triple is reifiable, i.e. if it is contextualized such that a reified representation
		 * is required in case a backend does not support contextualization of triples.
		 * @param triple the triple to check.
		 * @return true if the triple is reifiable.
		 */
		static bool isReifiable(const FramedTriple &triple);

	protected:
		semweb::VocabularyPtr vocabulary_;
		std::vector<FramedTriplePtr> reified_;
		knowrob::IRIAtomPtr name_;

		FramedTriplePtr &
		create(std::string_view subject, const AtomPtr &property, const std::optional<std::string_view> &g);
	};

	using ReifiedTriplePtr = std::shared_ptr<ReifiedTriple>;

} // knowrob

#endif //KNOWROB_REIFIED_TRIPLE_H
