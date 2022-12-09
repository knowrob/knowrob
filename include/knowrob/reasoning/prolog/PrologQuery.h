/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_QUERY_H__
#define __KNOWROB_PROLOG_QUERY_H__

// STD
#include <string>
#include <memory>
#include <map>
// SWI Prolog
#include <SWI-Prolog.h>
// KnowRob
#include <knowrob/lang/terms.h>
#include <knowrob/qa/queries.h>

namespace knowrob {
	/** Maps a Query object into term_t type of Prolog.
	 * Works in both directions.
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

		/** Returns the query as a term_t.
		 * @return the term_t reference.
		 */
		term_t& pl_query() { return pl_query_; }
		
		/** Returns the functor of the outermost term of the query as a predicate_t.
		 * @return the predicate_t reference.
		 */
		predicate_t& pl_predicate() { return pl_predicate_; }
		
		/** Returns the first argument of the outermost term of the query as a term_t.
		 * The term_t is a reference to an array of all arguments.
		 * @return the term_t reference.
		 */
		term_t& pl_arguments() { return pl_arguments_; }
		
		/** Returns the query object.
		 * @return the Query object
		 */
		const std::shared_ptr<Query>& qa_query() const { return qa_query_; }

		/** Get map of all variables in this query.
		 * @return the map.
		 */
		const std::map<std::string, term_t>& vars() const{ return vars_; }

		/** Translates a term_t reference to a Term object.
		 * @t a term_t reference.
		 * @return the Term object created.
		 */
		static std::shared_ptr<Term> constructTerm(const term_t &t);
		
	protected:
		std::shared_ptr<Query> qa_query_;
		
		term_t pl_query_;
		predicate_t pl_predicate_;
		term_t pl_arguments_;
		std::map<std::string, term_t> vars_;

		void constructPrologTerm(const std::shared_ptr<Term>& qa_term, term_t &pl_term);
		void constructPrologTerm(const std::shared_ptr<Formula>& phi, term_t &pl_term);
		void constructPrologTerm(ConnectiveFormula *phi, functor_t &pl_functor, term_t &pl_term);
		
		static std::shared_ptr<Predicate> constructPredicate(const term_t &t);
		static std::shared_ptr<Formula> constructFormula(const term_t &t);
		
		void createPrologPredicate();
	};
}

#endif //__KNOWROB_PROLOG_QUERY_H__

