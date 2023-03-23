//
// Created by daniel on 21.03.23.
//

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include "knowrob/queries/QueryParser.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/formulas/NegatedFormula.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/AtomicProposition.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

namespace qi = boost::spirit::qi;
// ASCII version of *char_*
namespace ascii = boost::spirit::ascii;

using Iterator = std::string::const_iterator;
// parser rules with different output types (second argument)
using TermRule      = qi::rule<Iterator, std::shared_ptr<Term>()>;
using PredicateRule = qi::rule<Iterator, std::shared_ptr<Predicate>()>;
using FormulaRule   = qi::rule<Iterator, std::shared_ptr<Formula>(), ascii::space_type>;
using StringRule    = qi::rule<Iterator, std::string()>;

namespace knowrob {
	struct ParserRules {
		// a rule that matches formulae of all kind
		FormulaRule formula;
		FormulaRule disjunction;
		FormulaRule conjunction;
		FormulaRule implication;
		FormulaRule negation;
		FormulaRule atomicProposition;
		FormulaRule modalFormula;
		FormulaRule belief;
		FormulaRule knowledge;
		FormulaRule oncePast;
		FormulaRule alwaysPast;
		FormulaRule brackets;

		// a rule that matches a single predicate
		PredicateRule predicate;

		// a rule that matches a single argument of a predicate
		TermRule argument;
		TermRule variable;
		TermRule constant;
		TermRule floating_point;
		TermRule integer32;
		TermRule integer64;
		TermRule atom;
		TermRule string;

		StringRule singleQuotes;
		StringRule doubleQuotes;
		StringRule lowerPrefix;
		StringRule upperPrefix;
	};
}

namespace {
	// code-snippet found on the web, needed to create smart pointers in parser rules.
	// effectively here `ptr_` is defined and can be used to create std::shared_ptr's
	// in parser rules.
	template <typename T> struct make_shared_f {
		template <typename... A> struct result { typedef std::shared_ptr<T> type; };

		template <typename... A> typename result<A...>::type operator()(A &&... a) const {
			return std::make_shared<T>(std::forward<A>(a)...);
		}
	};
	template <typename T> using ptr_ = boost::phoenix::function<make_shared_f<T> >;
}

QueryParser::QueryParser()
{
	bnf_ = new ParserRules();
	bnf_->singleQuotes %= qi::lexeme['\'' >> +(qi::char_ - '\'') >> '\''];
	bnf_->doubleQuotes %= qi::lexeme['"' >> +(qi::char_ - '"') >> '"'];
	bnf_->lowerPrefix  %= qi::lexeme[qi::lower >> *qi::char_];
	bnf_->upperPrefix  %= qi::lexeme[qi::upper >> *qi::char_];

	// atomic constants: strings, numbers etc.
	bnf_->atom = ((bnf_->lowerPrefix | bnf_->singleQuotes)
			[qi::_val = ptr_<StringTerm>()(qi::_1)]);
	bnf_->string = (bnf_->doubleQuotes		[qi::_val = ptr_<StringTerm>()(qi::_1)]);
	bnf_->floating_point = (qi::double_		[qi::_val = ptr_<DoubleTerm>()(qi::_1)]);
	bnf_->integer32      = (qi::int_		[qi::_val = ptr_<Integer32Term>()(qi::_1)]);
	bnf_->integer64      = (qi::long_		[qi::_val = ptr_<LongTerm>()(qi::_1)]);

	// arguments of predicates: constants or variables
	bnf_->constant %= (bnf_->atom
			| bnf_->string
			| bnf_->floating_point
			| bnf_->integer32
			| bnf_->integer64);
	bnf_->variable = (bnf_->upperPrefix [qi::_val = ptr_<Variable>()(qi::_1)]);
	bnf_->argument %= bnf_->variable | bnf_->constant;

	// predicates
	bnf_->predicate = (((bnf_->lowerPrefix | bnf_->singleQuotes) >>
			qi::char_('(') >> (bnf_->argument % ',') >> ')')
			[qi::_val = ptr_<Predicate>()(qi::_1, qi::_3)]);

	// formulas
	bnf_->brackets = ((qi::char_('(') >> bnf_->formula >> qi::char_('('))
			[qi::_val = qi::_2]);
	bnf_->atomicProposition = (bnf_->predicate
			[qi::_val = ptr_<AtomicProposition>()(qi::_1)]);
	// negation
	bnf_->negation = ((qi::char_('~') >> bnf_->formula)
			[qi::_val = ptr_<NegatedFormula>()(qi::_2)]);
	// implication
	bnf_->implication = ((bnf_->formula >> "->" >> bnf_->formula)
			[qi::_val = ptr_<Implication>()(qi::_1, qi::_2)]);
	// conjunction using ',' as operator symbol
	bnf_->conjunction = ((bnf_->formula % ',')
			[qi::_val = ptr_<Conjunction>()(qi::_1)]);
	// disjunction using ';' as operator symbol
	bnf_->disjunction = ((bnf_->formula % ';')
			[qi::_val = ptr_<Disjunction>()(qi::_1)]);
	// modal operators
	bnf_->belief = ((qi::char_('B') >> bnf_->formula)
			[qi::_val = ptr_<ModalFormula>()(BeliefModality::B(), qi::_2)]);
	bnf_->knowledge = ((qi::char_('K') >> bnf_->formula)
			[qi::_val = ptr_<ModalFormula>()(KnowledgeModality::K(), qi::_2)]);
	bnf_->oncePast = ((qi::char_('P') >> bnf_->formula)
			[qi::_val = ptr_<ModalFormula>()(PastModality::P(), qi::_2)]);
	bnf_->alwaysPast = ((qi::char_('H') >> bnf_->formula)
			[qi::_val = ptr_<ModalFormula>()(PastModality::H(), qi::_2)]);
	bnf_->modalFormula %= bnf_->belief
			| bnf_->knowledge
			| bnf_->oncePast
			| bnf_->alwaysPast;

	bnf_->formula %= bnf_->brackets
			| bnf_->negation
			| bnf_->modalFormula
			| bnf_->atomicProposition
			| bnf_->conjunction
			| bnf_->disjunction
			| bnf_->implication;
}

QueryParser::~QueryParser()
{
	delete bnf_;
}

FormulaPtr QueryParser::parse(const std::string &queryString)
{
	auto first = queryString.begin();
	auto last  = queryString.end();

	FormulaPtr formula;
	bool r = qi::phrase_parse(first, last,
							  bnf_->formula,
							  ascii::space,
							  formula);

	if (first == last && r) {
		return formula;
	}
	else {
		throw QueryError("Query string ({}) has invalid syntax.", queryString);
	}
}
