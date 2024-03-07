/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REIFIED_QUERY_H
#define KNOWROB_REIFIED_QUERY_H

#include "knowrob/queries/ConjunctiveQuery.h"
#include "knowrob/semweb/Vocabulary.h"

namespace knowrob {
	enum ReificationFlag {
		IncludeOriginal = 1 << 0,
		IncludeReified  = 1 << 1,
	};

	/**
	 * Reifies an inout query such that it can be issued to a backend that does not support
	 * contextualization of triples.
	 */
	class ReifiedQuery : public ConjunctiveQuery {
	public:
		/**
		 * @param nonReified a query that is not reified.
		 * @param vocabulary the vocabulary to use for reification.
		 */
		explicit ReifiedQuery(const std::shared_ptr<ConjunctiveQuery> &nonReified, semweb::VocabularyPtr vocabulary);

		/**
		 * @param nonReified a query that is not reified.
		 * @param ctx the query context.
		 * @param vocabulary the vocabulary to use for reification.
		 */
		explicit ReifiedQuery(const FramedTriplePattern &nonReified, semweb::VocabularyPtr vocabulary);

		/**
		 * @param q a triple query.
		 * @return a bitmask of ReificationFlag's that indicates if the pattern may have instances in the original or reified form.
		 */
		static int getReificationFlags(const FramedTriplePattern &q);

	protected:
		semweb::VocabularyPtr vocabulary_;

		void addNonReified(const FramedTriplePattern &nonReified);

		std::shared_ptr<FramedTriplePattern> create(const TermPtr &s, const TermPtr &p, const TermPtr &o, const groundable<Atom> &g);
	};

} // knowrob

#endif //KNOWROB_REIFIED_QUERY_H
