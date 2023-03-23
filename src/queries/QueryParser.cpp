//
// Created by daniel on 21.03.23.
//

#include <gtest/gtest.h>
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
using PredicateRule = qi::rule<Iterator, std::shared_ptr<Predicate>(), ascii::space_type>;
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
		TermRule number;
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
	bnf_->lowerPrefix  %= qi::lexeme[ascii::lower >> *ascii::alnum];
	bnf_->upperPrefix  %= qi::lexeme[ascii::upper >> *ascii::alnum];

	// atomic constants: strings, numbers etc.
	bnf_->atom = ((bnf_->lowerPrefix | bnf_->singleQuotes)
			[qi::_val = ptr_<StringTerm>()(qi::_1)]);
	bnf_->string = (bnf_->doubleQuotes
            [qi::_val = ptr_<StringTerm>()(qi::_1)]);
    bnf_->number = (qi::double_
            [qi::_val = ptr_<DoubleTerm>()(qi::_1)]);

	// arguments of predicates: constants or variables
	bnf_->constant %= (bnf_->atom
			| bnf_->string
            | bnf_->number);
	bnf_->variable = (bnf_->upperPrefix [qi::_val = ptr_<Variable>()(qi::_1)]);
	bnf_->argument %= bnf_->variable | bnf_->constant;

	// predicates
    // todo: add a case for nullary predicates
	bnf_->predicate = (((bnf_->lowerPrefix | bnf_->singleQuotes) >>
			qi::char_('(') >> (bnf_->argument % ',') >> ')')
			[qi::_val = ptr_<Predicate>()(qi::_1, qi::_3)]);

	// formulas
	bnf_->brackets = ((qi::char_('(') >> bnf_->formula >> qi::char_(')'))
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

template <typename ResultType, typename RuleType>
static inline ResultType parse_(const std::string &queryString, const RuleType &rule)
{
    auto first = queryString.begin();
    auto last  = queryString.end();

    ResultType result;
    bool r = qi::phrase_parse(first, last, rule, ascii::space, result);

    if (first == last && r) {
        return result;
    }
    else {
        throw QueryError("Query string ({}) has invalid syntax.", queryString);
    }
}

FormulaPtr QueryParser::parse(const std::string &queryString)
{
    return parse_<FormulaPtr,FormulaRule>(queryString, bnf_->formula);
}

PredicatePtr QueryParser::parsePredicate(const std::string &queryString)
{
    return parse_<PredicatePtr,PredicateRule>(queryString, bnf_->predicate);
}

// define a fixture class
class QueryParserTest : public ::testing::Test {
protected:
    // void SetUp() override {}
    // void TearDown() override {}
    QueryParser parser_;
};

TEST_F(QueryParserTest, ParsingPredicates)
{
    auto p2 = parser_.parsePredicate("p(X,a)");
    EXPECT_NE(p2.get(), nullptr);
    if(p2) {
        EXPECT_EQ(p2->indicator()->functor(), "p");
        EXPECT_EQ(p2->indicator()->arity(), 2);
        EXPECT_EQ(p2->arguments().size(), 2);
        if(p2->arguments().size() == 2) {
            EXPECT_EQ(p2->arguments()[0]->type(), TermType::VARIABLE);
            EXPECT_EQ(p2->arguments()[1]->type(), TermType::STRING);
        }
    }

    auto q2 = parser_.parsePredicate("q  (   3   ,    3.05   )");
    EXPECT_NE(p2.get(), nullptr);
    if(q2) {
        EXPECT_EQ(q2->indicator()->functor(), "q");
        EXPECT_EQ(q2->indicator()->arity(), 2);
        EXPECT_EQ(q2->arguments().size(), 2);
        if(q2->arguments().size() == 2) {
            EXPECT_EQ(q2->arguments()[0]->type(), TermType::DOUBLE);
            EXPECT_EQ(q2->arguments()[1]->type(), TermType::DOUBLE);
        }
    }
}
