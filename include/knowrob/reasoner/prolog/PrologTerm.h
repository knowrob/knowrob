/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_TERM_H
#define KNOWROB_PROLOG_TERM_H

#include <SWI-Prolog.h>
#include "knowrob/terms/Term.h"
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/CompoundFormula.h"
#include "knowrob/triples/FramedTriple.h"
#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/terms/Function.h"
#include "knowrob/terms/ListTerm.h"
#include <iostream>

namespace knowrob {
	/**
	 * A Prolog term. It can be constructed from several KnowRob datatypes and
	 * can be used to query a Prolog engine in a worker thread.
	 */
	class PrologTerm {
	public:
		/**
		 * @return the 'fail' atom.
		 */
		static const atom_t &ATOM_fail();

		/**
		 * @return the 'false' atom.
		 */
		static const atom_t &ATOM_false();

		/**
		 * @return the 'true' atom.
		 */
		static const atom_t &ATOM_true();

		/**
		 * @return the ',' atom.
		 */
		static const atom_t &ATOM_comma();

		/**
		 * @return the ';' atom.
		 */
		static const atom_t &ATOM_semicolon();

		/**
		 * @return the functor ','/2.
		 */
		static const functor_t &FUNCTOR_comma();

		/**
		 * @return the functor ';'/2.
		 */
		static const functor_t &FUNCTOR_semicolon();

		/**
		 * @return the comma predicate.
		 */
		static const predicate_t &PREDICATE_comma();

		/**
		 * @return the semicolon predicate.
		 */
		static const predicate_t &PREDICATE_semicolon();

		/**
		 * Generates an empty Prolog term.
		 */
		PrologTerm();

		/**
		 * Generates a Prolog term holding a KnowRob term.
		 * @param kbTerm a KnowRob term
		 */
		explicit PrologTerm(const TermPtr &kbTerm);

		/**
		 * Generates a Prolog term holding a KnowRob formula.
		 * @param kbFormula a KnowRob formula
		 */
		explicit PrologTerm(const FormulaPtr &kbFormula);

		/**
		 * @param literal an RDF literal
		 */
		explicit PrologTerm(const FramedTriplePattern &literal);

		/**
		 * Generates a Prolog term holding a KnowRob triple.
		 * @param functor the functor of the term
		 * @param triple a triple
		 */
		PrologTerm(const FramedTriple &triple, std::string_view functor);

		/**
		 * Generates a Prolog term holding a list of terms.
		 * @param args the list of terms
		 * @param functor the functor of the term
		 */
		PrologTerm(const std::vector<PrologTerm> &args, std::string_view functor);

		/**
		 * Generates a Prolog term holding a list of terms.
		 * @tparam Args anything that can be converted to a PrologTerm with a single-argument constructor.
		 * @param functor the functor of the term
		 * @param args the list of terms
		 */
		template<typename ... Args>
		explicit PrologTerm(std::string_view functor, Args &&... args)
				: PrologTerm(readArgs(std::forward<Args>(args)...), functor) {}

		/**
		 * Copies a term handle.
		 */
		PrologTerm(const PrologTerm &other);

		/**
		 * @return the term handle.
		 */
		term_t operator()() const { return plTerm_; }

		/**
		 * @param other a term handle
		 * @return the conjunction of the two terms
		 */
		PrologTerm operator&(const PrologTerm &other) const;

		/**
		 * @param module the module to query
		 */
		void setModule(std::string_view module) { module_ = module; }

		/**
		 * @return the module to query
		 */
		auto module() const { return module_; }

		/**
		 * @param functor the functor of the term
		 * @param triple a triple
		 * @return true if the term was assigned to the triple
		 */
		bool putTriple(std::string_view functor, const FramedTriple &triple);

		/**
		 * @param kbTerm a KnowRob term
		 * @return true if the term was assigned to the KnowRob term
		 */
		bool putTerm(const TermPtr &kbTerm);

		/**
		 * @param kbFormula a KnowRob formula
		 * @return true if the term was assigned to the KnowRob formula
		 */
		bool putFormula(const FormulaPtr &phi);

		/**
		 * @return true if the term is ground
		 */
		bool isGround() const { return vars_.empty(); }

		/**
		 * @return the variables of the term
		 */
		auto &vars() const { return vars_; }

		/**
		 * @param flags the flags to open the query
		 * @return the query id
		 */
		qid_t openQuery(int flags) const;

		/**
		 * If this call succeeds, then vars() should hold the instantiations of the variables.
		 * @param qid the query id
		 * @return true if the query has a next solution
		 */
		bool nextSolution(qid_t qid) const;

		/**
		 * @return translates the term to a KnowRob term
		 */
		TermPtr toKnowRobTerm() const;

		/**
		 * @return translates the term to a KnowRob formula
		 */
		FormulaPtr toKnowRobFormula() const;

		/**
		 * @param t a Prolog term
		 * @return translates the term to a KnowRob term
		 */
		static TermPtr toKnowRobTerm(const term_t &t);

		/**
		 * @param t a Prolog term
		 * @return translates the term to a KnowRob formula
		 */
		static FormulaPtr toKnowRobFormula(const TermPtr &t);

		/**
		 * @param plTerm a Prolog variable term
		 * @return the name of the variable
		 */
		static std::string getVarName(term_t plTerm);

		/**
		 * @return a term representing the empty list
		 */
		static PrologTerm nil();

	protected:
		std::map<std::string, term_t, std::less<>> vars_;
		std::optional<std::string_view> module_;
		term_t plTerm_;

		void readVars(term_t plTerm);

		bool putFormula(const FormulaPtr &phi, term_t plTerm);

		bool putFunction(Function *fn, term_t pl_term);

		bool putList(ListTerm *list, term_t pl_term);

		bool putTerm(const TermPtr &kbTerm, term_t plTerm);

		bool putCompound(CompoundFormula *phi, term_t pl_term, const functor_t &pl_functor);

		template<typename ... Args>
		std::vector<PrologTerm> readArgs(Args &&... args) {
			// use a fold expression to convert the arguments to a vector
			// @see https://en.cppreference.com/w/cpp/language/fold
			std::vector<PrologTerm> argTerms;
			([&args, &argTerms] { argTerms.push_back(PrologTerm(args)); }(), ...);
			return argTerms;
		}

		friend class PrologList;
	};

	class PrologList : public PrologTerm {
	public:
		explicit PrologList(const std::vector<PrologTerm> &elements);
	};

} // knowrob

#endif //KNOWROB_PROLOG_TERM_H
