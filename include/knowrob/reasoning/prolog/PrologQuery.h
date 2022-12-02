/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_REASONER_H__
#define __KNOWROB_PROLOG_REASONER_H__

// STD
#include <string>
#include <memory>
// KnowRob
#include <knowrob/lang/terms.h>
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/LogicProgramReasoner.h>
#include <knowrob/reasoning/prolog/PrologPool.h>

namespace knowrob {
	/** Maps a Query object into term_t type of Prolog.
	 */
	class PrologQuery {
	public:
		/**
		 * @query a Query object.
		 */
		PrologQuery(const std::shared_ptr<Query> &query);
		/**
		 * @queryString a string encoded in Prolog syntax.
		 */
		PrologQuery(const std::string &queryString);
		/**
		 * @plQuery a Prolog term.
		 */
		PrologQuery(const term_t &plQuery);
		
		// copy not supported
		PrologQuery(const PrologQuery&) = delete;
		
		~PrologQuery();

		/** Get the query term.
		 * @return the term.
		 */
		const term_t& getPrologTerm() const { return pl_query_; }
		
		/**
		 */
		const std::shared_ptr<Query>& getQuery() const { return qa_query_; }

		/** Get map of all variables in this query.
		 * @return the map.
		 */
		const std::map<std::string, term_t>& vars() const{ return vars_; }

	protected:
		std::shared_ptr<Query> qa_query_;
		term_t pl_query_;
		std::map<std::string, term_t> vars_;

		void constructPrologTerm(const std::shared_ptr<Term>& qa_term, term_t &pl_term);
		void constructPrologTerm(const std::shared_ptr<Formula>& phi, term_t &pl_term);
		void constructPrologTerm(ConnectiveFormula *phi, functor_t &pl_functor, term_t &pl_term);
		
		std::shared_ptr<Formula> constructFormula(const term_t &t);
		std::shared_ptr<Term> constructTerm(const term_t &t);
		std::shared_ptr<Predicate> constructPredicate(const term_t &t);
	};
}

#endif //__KNOWROB_PROLOG_REASONER_H__
