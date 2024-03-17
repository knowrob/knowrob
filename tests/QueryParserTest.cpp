/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <knowrob/queries/QueryParser.h>
#include "knowrob/knowrob.h"
#include "knowrob/formulas/CompoundFormula.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/terms/String.h"

// fixture class for testing
namespace knowrob::testing {
	class QueryParserTest : public ::testing::Test {
	protected:
		// void SetUp() override { }
		// void TearDown() override {}
	};
}
using namespace knowrob::testing;
using namespace knowrob;

static inline void testNumber(const TermPtr &t, const double &expected) {
    EXPECT_NE(t.get(), nullptr);
    if (t) {
        EXPECT_EQ(t->termType(), TermType::ATOMIC);
        if(t->termType() == TermType::ATOMIC) {
			auto *a = (Atomic*) t.get();
			EXPECT_EQ(a->atomicType(), AtomicType::NUMERIC);
		}
		std::ostringstream oss;
		oss << expected;
		EXPECT_EQ(readString(*t), oss.str());
    }
}

template<typename ConstantType, typename PrimitiveType>
static inline void testConstant(const TermPtr &t, const AtomicType &atomicType, const std::string &expected) {
    EXPECT_NE(t.get(), nullptr);
    if (t) {
        EXPECT_EQ(t->termType(), TermType::ATOMIC);
        if(t->termType() == TermType::ATOMIC) {
			auto *a = (Atomic*) t.get();
			EXPECT_EQ(a->atomicType(), atomicType);
		}
		EXPECT_EQ(readString(*t), expected);
    }
}

static inline void testAtom(const TermPtr &t, const std::string &expected) {
    testConstant<Atom, std::string>(t, AtomicType::ATOM, expected);
}

static inline void testString(const TermPtr &t, const std::string &expected) {
    testConstant<String, std::string>(t, AtomicType::STRING, expected);
}

static inline void testPredicate(
        const PredicatePtr &p,
        const std::string &expectedFunctor,
        int expectedArity,
        std::vector<TermType> expectedTypes) {
    EXPECT_NE(p.get(), nullptr);
    if (p) {
        EXPECT_EQ(p->functor()->stringForm(), expectedFunctor);
        EXPECT_EQ(p->arity(), expectedArity);
        EXPECT_EQ(p->arguments().size(), expectedArity);
        if (p->arguments().size() == expectedArity) {
            for (int i = 0; i < expectedArity; ++i) {
                EXPECT_EQ(p->arguments()[i]->termType(), expectedTypes[i]);
            }
        }
    }
}

static inline void testCompound(const FormulaType &phiType, const FormulaPtr &phi,
                                const int numArgs, std::vector<FormulaType> argTypes) {
    EXPECT_NE(phi.get(), nullptr);
    if (phi) {
        EXPECT_EQ(phi->type(), phiType);
        if (phi->type() == phiType) {
            auto *phi1 = (CompoundFormula *) phi.get();
            EXPECT_EQ(phi1->formulae().size(), numArgs);
            if (phi1->formulae().size() == numArgs) {
                for (int i = 0; i < numArgs; ++i) {
                    EXPECT_EQ(phi1->formulae()[i]->type(), argTypes[i]);
                }
            }
        }
    }
}

static inline void testModal(const FormulaPtr &phi, const std::string &op, const FormulaType &argType) {
    EXPECT_NE(phi.get(), nullptr);
    if (phi) {
        EXPECT_EQ(phi->type(), FormulaType::MODAL);
        if (phi->type() == FormulaType::MODAL) {
            auto *m = (ModalFormula *) phi.get();
            EXPECT_EQ(std::string(m->modalOperator()->symbol()), op);

            EXPECT_NE(m->modalFormula().get(), nullptr);
            if (m->modalFormula()) {
                EXPECT_EQ(m->modalFormula()->type(), argType);
            }
        }
    }
}

#define TEST_NO_THROW(Arg) { SCOPED_TRACE("QueryParserTest");  EXPECT_NO_THROW(Arg); }

TEST_F(QueryParserTest, Numbers) {
    TEST_NO_THROW(testNumber(QueryParser::parseConstant("234"), 234.0))
    TEST_NO_THROW(testNumber(QueryParser::parseConstant("-45"), -45.0))
    TEST_NO_THROW(testNumber(QueryParser::parseConstant("-45.64"), -45.64))
}

TEST_F(QueryParserTest, RawAtoms) {
    TEST_NO_THROW(EXPECT_EQ(QueryParser::parseRawAtom("p"), "p"))
    TEST_NO_THROW(EXPECT_EQ(QueryParser::parseRawAtom("p2"), "p2"))
    TEST_NO_THROW(EXPECT_EQ(QueryParser::parseRawAtom("p_2"), "p_2"))
    TEST_NO_THROW(EXPECT_EQ(QueryParser::parseRawAtom("'Foo'"), "Foo"))
    TEST_NO_THROW(EXPECT_EQ(QueryParser::parseRawAtom("owl:foo"), "http://www.w3.org/2002/07/owl#foo"))
}

TEST_F(QueryParserTest, Atoms) {
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("p"), "p"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("p2"), "p2"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("pSDd2"), "pSDd2"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("'Foo'"), "'Foo'"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("'x#/&%s'"), "x#/&%s"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("owl:foo"), "http://www.w3.org/2002/07/owl#foo"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("owl:Foo"), "http://www.w3.org/2002/07/owl#Foo"))
    TEST_NO_THROW(testAtom(QueryParser::parseConstant("owl:'Foo'"), "http://www.w3.org/2002/07/owl#Foo"))
}

TEST_F(QueryParserTest, Strings) {
    TEST_NO_THROW(testString(QueryParser::parseConstant("\"Foo\""), "\"Foo\""))
    TEST_NO_THROW(testString(QueryParser::parseConstant("\"x#/&%s\""), "\"x#/&%s\""))
}

TEST_F(QueryParserTest, InvalidConstant) {
    EXPECT_THROW(QueryParser::parseConstant("X1"), QueryError);
    EXPECT_THROW(QueryParser::parseConstant("p(x)"), QueryError);
    EXPECT_THROW(QueryParser::parseConstant("p,q"), QueryError);
}

TEST_F(QueryParserTest, Predicates) {
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("p(X,a)"),
            "p", 2, {TermType::VARIABLE, TermType::ATOMIC}))
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("p(?x,a)"),
            "p", 2, {TermType::VARIABLE, TermType::ATOMIC}))
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("'X1'(x1)"),
            "X1", 1, {TermType::ATOMIC}))
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("q  (   3   ,    \"x\"   )"),
            "q", 2, {TermType::ATOMIC, TermType::ATOMIC}))
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("nullary"),
            "nullary", 0, {}))
}

TEST_F(QueryParserTest, PredicateWithCompundArgument) {
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("p(X,'<'(a))"),
            "p", 2, {TermType::VARIABLE, TermType::FUNCTION}));
    TEST_NO_THROW(testPredicate(
            QueryParser::parsePredicate("p(X,[a,b])"),
            "p", 2, {TermType::VARIABLE, TermType::FUNCTION}));
}

TEST_F(QueryParserTest, InvalidPredicates) {
    EXPECT_THROW(QueryParser::parsePredicate("X1"), QueryError);
    EXPECT_THROW(QueryParser::parsePredicate("2"), QueryError);
    EXPECT_THROW(QueryParser::parsePredicate("p,q"), QueryError);
}

TEST_F(QueryParserTest, Negations) {
    TEST_NO_THROW(testCompound(FormulaType::NEGATION,
                               QueryParser::parse("~p"),
                               1, {FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::NEGATION,
                               QueryParser::parse("~(p&q)"),
                               1, {FormulaType::CONJUNCTION}))
}

TEST_F(QueryParserTest, Conjunctions) {
    TEST_NO_THROW(testCompound(FormulaType::CONJUNCTION,
                               QueryParser::parse("p,q"),
                               2, {FormulaType::PREDICATE, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::CONJUNCTION,
                               QueryParser::parse("  p,   q  &  r  "),
                               3, {FormulaType::PREDICATE, FormulaType::PREDICATE, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::CONJUNCTION,
                               QueryParser::parse("p,(q;r)"),
                               2, {FormulaType::PREDICATE, FormulaType::DISJUNCTION}))
    TEST_NO_THROW(testCompound(FormulaType::CONJUNCTION,
                               QueryParser::parse("(p|q)&r"),
                               2, {FormulaType::DISJUNCTION, FormulaType::PREDICATE}))
}

TEST_F(QueryParserTest, Disjunctions) {
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("p;q"),
                               2, {FormulaType::PREDICATE, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("  p;   q  |  r  "),
                               3, {FormulaType::PREDICATE, FormulaType::PREDICATE, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("p;(q,r)"),
                               2, {FormulaType::PREDICATE, FormulaType::CONJUNCTION}))
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("(p,q);r"),
                               2, {FormulaType::CONJUNCTION, FormulaType::PREDICATE}))
}

TEST_F(QueryParserTest, Implications) {
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("p->q"),
                               2, {FormulaType::PREDICATE, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("  p->    q  ->  r  "),
                               2, {FormulaType::PREDICATE, FormulaType::IMPLICATION}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("p->(q,r)"),
                               2, {FormulaType::PREDICATE, FormulaType::CONJUNCTION}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("(p,q)->r"),
                               2, {FormulaType::CONJUNCTION, FormulaType::PREDICATE}))
}

TEST_F(QueryParserTest, ModalFormulas) {
    TEST_NO_THROW(testModal(QueryParser::parse("B p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B p"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("Bp"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B(p)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("Kq(a)"), "K", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("BBq"), "B", FormulaType::MODAL))
    TEST_NO_THROW(testModal(QueryParser::parse("B (b,q)"), "B", FormulaType::CONJUNCTION))
}

TEST_F(QueryParserTest, ModalityWithArguments) {
    TEST_NO_THROW(testModal(QueryParser::parse("B[self] p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B['self'] p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B[fred,confidence=0.8] p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B[fred,0.8] p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B[0.8,fred] p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B[0.8] p(x)"), "B", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("B[confidence=0.8] p(x)"), "B", FormulaType::PREDICATE))

    TEST_NO_THROW(testModal(QueryParser::parse("P[begin=10,end=20] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[begin=10] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[end=20] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[until=20] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[10.0,20.0] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[10.0] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[10,20] p(x)"), "P", FormulaType::PREDICATE))
}

TEST_F(QueryParserTest, ModalityWithWrongArguments) {
    EXPECT_THROW(testModal(QueryParser::parse("B[foo=fred] p(x)"), "B", FormulaType::PREDICATE), QueryError);
    EXPECT_THROW(testModal(QueryParser::parse("B[0.8,0.8] p(x)"), "B", FormulaType::PREDICATE), QueryError);
}

TEST_F(QueryParserTest, ModalityWithEmptyArguments) {
	//GTEST_SKIP() << "syntax like P[,10.0] where an argument is missing is not supported yet by the query parser.";
    TEST_NO_THROW(testModal(QueryParser::parse("P[_,10.0] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[10,_] p(x)"), "P", FormulaType::PREDICATE))
}

TEST_F(QueryParserTest, Precedence) {
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("p;q,r"),
                               2, {FormulaType::PREDICATE, FormulaType::CONJUNCTION}))
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("p,q;r"),
                               2, {FormulaType::CONJUNCTION, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::DISJUNCTION,
                               QueryParser::parse("Bp;r"),
                               2, {FormulaType::MODAL, FormulaType::PREDICATE}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("p,q->r;p"),
                               2, {FormulaType::CONJUNCTION, FormulaType::DISJUNCTION}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("p,q->r->p"),
                               2, {FormulaType::CONJUNCTION, FormulaType::IMPLICATION}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("Bp->Kp"),
                               2, {FormulaType::MODAL, FormulaType::MODAL}))
    TEST_NO_THROW(testCompound(FormulaType::IMPLICATION,
                               QueryParser::parse("Bp->~p"),
                               2, {FormulaType::MODAL, FormulaType::NEGATION}))
}
