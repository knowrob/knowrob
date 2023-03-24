//
// Created by daniel on 21.03.23.
//

#include <gtest/gtest.h>

//#define BOOST_SPIRIT_DEBUG
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
namespace px = boost::phoenix;
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
		FormulaRule atomic;
		FormulaRule modalFormula;
		FormulaRule belief;
		FormulaRule knowledge;
		FormulaRule oncePast;
		FormulaRule alwaysPast;
		FormulaRule brackets;
        FormulaRule unary;

		// a rule that matches a single predicate
		PredicateRule predicate;
        PredicateRule predicateNullary;
        PredicateRule predicateWithArgs;

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
	bnf_->predicateWithArgs = (((bnf_->lowerPrefix | bnf_->singleQuotes) >>
			qi::char_('(') >> (bnf_->argument % ',') >> ')')
			[qi::_val = ptr_<Predicate>()(qi::_1, qi::_3)]);
    bnf_->predicateNullary = ((bnf_->lowerPrefix | bnf_->singleQuotes)
            [qi::_val = ptr_<Predicate>()(qi::_1, std::vector<TermPtr>())]);
    bnf_->predicate %= bnf_->predicateWithArgs | bnf_->predicateNullary;

	// formulas
	bnf_->brackets = (('(' >> bnf_->formula >> ')') [qi::_val = qi::_1]);
	bnf_->atomic = (bnf_->predicate                 [qi::_val = ptr_<AtomicProposition>()(qi::_1)]);

	// unary operators
	bnf_->negation   = (('~' >> (bnf_->unary | bnf_->brackets))
			[qi::_val = ptr_<NegatedFormula>()(qi::_1)]);
	bnf_->belief     = (('B' >> (bnf_->unary | bnf_->brackets))
			[qi::_val = ptr_<ModalFormula>()(BeliefModality::B(), qi::_1)]);
	bnf_->knowledge  = (('K' >> (bnf_->unary | bnf_->brackets))
			[qi::_val = ptr_<ModalFormula>()(KnowledgeModality::K(), qi::_1)]);
	bnf_->oncePast   = (('P' >> (bnf_->unary | bnf_->brackets))
			[qi::_val = ptr_<ModalFormula>()(PastModality::P(), qi::_1)]);
	bnf_->alwaysPast = (('H' >> (bnf_->unary | bnf_->brackets))
			[qi::_val = ptr_<ModalFormula>()(PastModality::H(), qi::_1)]);
	bnf_->modalFormula %= bnf_->belief
			| bnf_->knowledge
			| bnf_->oncePast
			| bnf_->alwaysPast;
    bnf_->unary %= bnf_->modalFormula | bnf_->negation | bnf_->atomic;

    // compound formulae
    bnf_->conjunction = (((bnf_->unary | bnf_->brackets)
            >> (qi::char_(',')|qi::char_('&'))
            >> bnf_->formula)
            [ qi::_val = (qi::_1 & qi::_3) ]
            | bnf_->unary [qi::_val = qi::_1] );
    bnf_->disjunction = (((bnf_->conjunction | bnf_->brackets)
            >> (qi::char_(';')|qi::char_('|'))
            >> bnf_->formula)
            [ qi::_val = (qi::_1 | qi::_3) ]
            | bnf_->conjunction [qi::_val = qi::_1] );
    bnf_->implication = (((bnf_->disjunction | bnf_->brackets)
            >> "->"
            >> bnf_->formula)
            [ qi::_val = ptr_<Implication>()(qi::_1, qi::_2) ]
            | bnf_->disjunction [qi::_val = qi::_1] );

	bnf_->formula %= bnf_->implication | ('(' >> bnf_->formula >> ')');
    //BOOST_SPIRIT_DEBUG_NODES((bnf_->conjunction)(bnf_->formula))
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

TermPtr QueryParser::parseConstant(const std::string &queryString)
{
    return parse_<TermPtr,TermRule>(queryString, bnf_->constant);
}

// define a fixture class
class QueryParserTest : public ::testing::Test {
protected:
    QueryParser parser_;

    // void SetUp() override { }
    // void TearDown() override {}
};

static inline void testNumber(const TermPtr &t, const double &expected)
{
    EXPECT_NE(t.get(), nullptr);
    if(t) {
        EXPECT_EQ(t->type(), TermType::DOUBLE);
        if(t->type() == TermType::DOUBLE) {
            EXPECT_DOUBLE_EQ(((DoubleTerm*)t.get())->value(), expected);
        }
    }
}

template <typename ConstantType, typename PrimitiveType>
static inline void testConstant(const TermPtr &t, const TermType &termType, const PrimitiveType &expected)
{
    EXPECT_NE(t.get(), nullptr);
    if(t) {
        EXPECT_EQ(t->type(), termType);
        if(t->type() == termType) {
            EXPECT_EQ(((ConstantType*)t.get())->value(), expected);
        }
    }
}
static inline void testAtom(const TermPtr &t, const std::string &expected)
{ testConstant<StringTerm,std::string>(t, TermType::STRING, expected); }
static inline void testString(const TermPtr &t, const std::string &expected)
{ testConstant<StringTerm,std::string>(t, TermType::STRING, expected); }

static inline void testPredicate(
        const PredicatePtr &p,
        const std::string &expectedFunctor,
        int expectedArity,
        std::vector<TermType> expectedTypes)
{
    EXPECT_NE(p.get(), nullptr);
    if(p) {
        EXPECT_EQ(p->indicator()->functor(), expectedFunctor);
        EXPECT_EQ(p->indicator()->arity(), expectedArity);
        EXPECT_EQ(p->arguments().size(), expectedArity);
        if(p->arguments().size() == expectedArity) {
            for(int i=0; i<expectedArity; ++i) {
                EXPECT_EQ(p->arguments()[i]->type(), expectedTypes[i]);
            }
        }
    }
}

static inline void testCompound(const FormulaType &phiType, const FormulaPtr &phi,
                                const int numArgs, std::vector<FormulaType> argTypes)
{
    EXPECT_NE(phi.get(), nullptr);
    if(phi) {
        EXPECT_EQ(phi->type(), phiType);
        if(phi->type() == phiType) {
            auto *phi1 = (CompoundFormula*)phi.get();
            EXPECT_EQ(phi1->formulae().size(), numArgs);
            if(phi1->formulae().size() == numArgs) {
                for(int i=0; i<numArgs; ++i) {
                    EXPECT_EQ(phi1->formulae()[i]->type(), argTypes[i]);
                }
            }
        }
    }
}

static inline void testModal(const FormulaPtr &phi, const std::string &op, const FormulaType &argType)
{
    EXPECT_NE(phi.get(), nullptr);
    if(phi) {
        EXPECT_EQ(phi->type(), FormulaType::MODAL);
        if(phi->type() == FormulaType::MODAL) {
            auto *m = (ModalFormula*)phi.get();
            EXPECT_EQ(std::string(m->modalOperator().symbol()), op);

            EXPECT_NE(m->modalFormula().get(), nullptr);
            if(m->modalFormula()) {
                EXPECT_EQ(m->modalFormula()->type(), argType);
            }
        }
    }
}

#define _NO_THROW(Arg) { SCOPED_TRACE("QueryParserTest");  EXPECT_NO_THROW(Arg); }

TEST_F(QueryParserTest, Numbers)
{
    _NO_THROW(testNumber(parser_.parseConstant("234"), 234.0))
    _NO_THROW(testNumber(parser_.parseConstant("-45"), -45.0))
    _NO_THROW(testNumber(parser_.parseConstant("-45.64"), -45.64))
}

TEST_F(QueryParserTest, Atoms)
{
    _NO_THROW(testAtom(parser_.parseConstant("p"), "p"))
    _NO_THROW(testAtom(parser_.parseConstant("p2"), "p2"))
    _NO_THROW(testAtom(parser_.parseConstant("pSDd2"), "pSDd2"))
    _NO_THROW(testAtom(parser_.parseConstant("'Foo'"), "Foo"))
    _NO_THROW(testAtom(parser_.parseConstant("'x#/&%s'"), "x#/&%s"))
}

TEST_F(QueryParserTest, Strings)
{
    _NO_THROW(testString(parser_.parseConstant("\"Foo\""), "Foo"))
    _NO_THROW(testString(parser_.parseConstant("\"x#/&%s\""), "x#/&%s"))
}

TEST_F(QueryParserTest, InvalidConstant)
{
    EXPECT_THROW(parser_.parseConstant("X1"), QueryError);
    EXPECT_THROW(parser_.parseConstant("p(x)"), QueryError);
    EXPECT_THROW(parser_.parseConstant("p,q"), QueryError);
}

TEST_F(QueryParserTest, Predicates)
{
    _NO_THROW(testPredicate(
            parser_.parsePredicate("p(X,a)"),
            "p", 2, { TermType::VARIABLE, TermType::STRING }))
    _NO_THROW(testPredicate(
            parser_.parsePredicate("'X1'(x1)"),
            "X1", 1, { TermType::STRING }))
    _NO_THROW(testPredicate(
            parser_.parsePredicate("q  (   3   ,    \"x\"   )"),
            "q", 2, { TermType::DOUBLE, TermType::STRING }))
    _NO_THROW(testPredicate(
            parser_.parsePredicate("nullary"),
            "nullary", 0, { }))
}

TEST_F(QueryParserTest, InvalidPredicates)
{
    EXPECT_THROW(parser_.parsePredicate("X1"), QueryError);
    EXPECT_THROW(parser_.parsePredicate("2"), QueryError);
    EXPECT_THROW(parser_.parsePredicate("p,q"), QueryError);
}

TEST_F(QueryParserTest, Conjunctions)
{
    _NO_THROW(testCompound(FormulaType::CONJUNCTION,
                           parser_.parse("p,q"),
                           2, { FormulaType::PREDICATE, FormulaType::PREDICATE }))
    _NO_THROW(testCompound(FormulaType::CONJUNCTION,
                           parser_.parse("  p,   q  ,  r  "),
                           3, { FormulaType::PREDICATE, FormulaType::PREDICATE, FormulaType::PREDICATE }))
    _NO_THROW(testCompound(FormulaType::CONJUNCTION,
                           parser_.parse("p,(q;r)"),
                           2, { FormulaType::PREDICATE, FormulaType::DISJUNCTION }))
    _NO_THROW(testCompound(FormulaType::CONJUNCTION,
                           parser_.parse("(p;q),r"),
                           2, { FormulaType::DISJUNCTION, FormulaType::PREDICATE }))
}

TEST_F(QueryParserTest, Disjunctions)
{
    _NO_THROW(testCompound(FormulaType::DISJUNCTION,
                           parser_.parse("p;q"),
                           2, { FormulaType::PREDICATE, FormulaType::PREDICATE }))
    _NO_THROW(testCompound(FormulaType::DISJUNCTION,
                           parser_.parse("  p;   q  ;  r  "),
                           3, { FormulaType::PREDICATE, FormulaType::PREDICATE, FormulaType::PREDICATE }))
    _NO_THROW(testCompound(FormulaType::DISJUNCTION,
                           parser_.parse("p;(q,r)"),
                           2, { FormulaType::PREDICATE, FormulaType::CONJUNCTION }))
    _NO_THROW(testCompound(FormulaType::DISJUNCTION,
                           parser_.parse("(p,q);r"),
                           2, { FormulaType::CONJUNCTION, FormulaType::PREDICATE }))
}

TEST_F(QueryParserTest, Implications)
{
    _NO_THROW(testCompound(FormulaType::IMPLICATION,
                           parser_.parse("p->q"),
                           2, { FormulaType::PREDICATE, FormulaType::PREDICATE }))
    _NO_THROW(testCompound(FormulaType::IMPLICATION,
                           parser_.parse("  p->   q  ->  r  "),
                           2, { FormulaType::PREDICATE, FormulaType::IMPLICATION }))
    _NO_THROW(testCompound(FormulaType::IMPLICATION,
                           parser_.parse("p->(q,r)"),
                           2, { FormulaType::PREDICATE, FormulaType::CONJUNCTION }))
    _NO_THROW(testCompound(FormulaType::IMPLICATION,
                           parser_.parse("(p,q)->r"),
                           2, { FormulaType::CONJUNCTION, FormulaType::PREDICATE }))
}

TEST_F(QueryParserTest, ModalFormulas)
{
    _NO_THROW(testModal(parser_.parse("B p(x)"), "B", FormulaType::PREDICATE))
    _NO_THROW(testModal(parser_.parse("B p"),    "B", FormulaType::PREDICATE))
    _NO_THROW(testModal(parser_.parse("Bp"),     "B", FormulaType::PREDICATE))
    _NO_THROW(testModal(parser_.parse("B(p)"),   "B", FormulaType::PREDICATE))
    _NO_THROW(testModal(parser_.parse("Kq(a)"),  "K", FormulaType::PREDICATE))
    _NO_THROW(testModal(parser_.parse("BBq"),    "B", FormulaType::MODAL))
    _NO_THROW(testModal(parser_.parse("B (b,q)"), "B", FormulaType::CONJUNCTION))
}

TEST_F(QueryParserTest, Precedence)
{
    _NO_THROW(testCompound(FormulaType::DISJUNCTION,
                           parser_.parse("p;q,r"),
                           2, { FormulaType::PREDICATE, FormulaType::CONJUNCTION }))
    _NO_THROW(testCompound(FormulaType::DISJUNCTION,
                           parser_.parse("p,q;r"),
                           2, { FormulaType::CONJUNCTION, FormulaType::PREDICATE }))
    _NO_THROW(testCompound(FormulaType::IMPLICATION,
                           parser_.parse("p,q->r;p"),
                           2, { FormulaType::CONJUNCTION, FormulaType::DISJUNCTION }))
    _NO_THROW(testCompound(FormulaType::IMPLICATION,
                           parser_.parse("p,q->r->p"),
                           2, { FormulaType::CONJUNCTION, FormulaType::IMPLICATION }))
}
