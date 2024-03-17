/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>

//#define BOOST_SPIRIT_DEBUG
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include "knowrob/queries/QueryParser.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/terms/OptionList.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

namespace qi = boost::spirit::qi;
namespace px = boost::phoenix;
// ASCII version of *char_*
namespace ascii = boost::spirit::ascii;

using Iterator = std::string::const_iterator;
// parser rules with different output types (second argument)
using TermRule = qi::rule<Iterator, std::shared_ptr<Term>(), ascii::space_type>;
using PredicateRule = qi::rule<Iterator, std::shared_ptr<Predicate>(), ascii::space_type>;
using FunctionRule = qi::rule<Iterator, std::shared_ptr<Function>(), ascii::space_type>;
using FormulaRule = qi::rule<Iterator, std::shared_ptr<Formula>(), ascii::space_type>;
using AtomRule = qi::rule<Iterator, std::shared_ptr<Atom>(), ascii::space_type>;
using StringRule = qi::rule<Iterator, std::string()>;

namespace knowrob {
    struct ParserRules {
        // a rule that matches formulae of all kind
        FormulaRule formula;
        FormulaRule disjunction;
        FormulaRule conjunction;
        FormulaRule implication;
        FormulaRule negation;
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
        TermRule term;
        FunctionRule function;
        TermRule variable, variable1, variable2;
        TermRule atomic;
        TermRule list;
        TermRule number;
        TermRule string;
        TermRule option;
        //TermRule optionFlag;
        TermRule keyvalue;
        TermRule options;
        TermRule nil;

        AtomRule atom;
		AtomRule atom_regular;
        AtomRule iri_atom;


        StringRule singleQuotes;
        StringRule doubleQuotes;
        StringRule lowerPrefix;
        StringRule upperPrefix;
        StringRule atomRaw;
        StringRule atomRawWithIRI;

        StringRule iri_raw;
        StringRule iri_ns;
        StringRule iri_entity;
    };
}

namespace {
    // code-snippet found on the web, needed to create smart pointers in parser rules.
    // effectively here `ptr_` is defined and can be used to create std::shared_ptr's
    // in parser rules.
    template<typename T>
    struct make_shared_f {
        template<typename... A>
        struct result {
            typedef std::shared_ptr<T> type;
        };

        template<typename... A>
        typename result<A...>::type operator()(A &&... a) const {
            return std::make_shared<T>(std::forward<A>(a)...);
        }
    };

    template<typename T> using ptr_ = boost::phoenix::function<make_shared_f<T> >;
}

static std::string createIRI(const std::string &prefix, const std::string &name) {
    auto uri = PrefixRegistry::createIRI(prefix, name);
    if (uri.has_value()) {
        return uri.value();
    } else {
        throw QueryError("Cannot construct IRI for '{}': "
                         "IRI prefix '{}' is not registered!", name, prefix);
    }
}

static std::optional<TimePoint> getBeginOption(const OptionList &options) {
    return options.contains("begin") ? options.getDouble("begin") : std::nullopt;
}

static std::optional<TimePoint> getEndOption(const OptionList &options) {
    return options.contains("end") ? options.getDouble("end") : std::nullopt;
}

static void reportUnrecognized(const TermPtr &option) {
    throw QueryError("Unrecognized option ({}) in modal operator.", *option);
}

static ModalOperatorPtr createK(const TermPtr &optionsTerm)
{
	static const auto a_agent1 = Atom::Tabled("agent");
	static const auto a_agent2 = Atom::Tabled("a");

    if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
        auto listTerm = (ListTerm*)optionsTerm.get();
        std::optional<std::string> agentName;

        for(auto &option : *listTerm) {
            // handle options without key
            if(!agentName.has_value() && option->isAtom()) {
                // assume first string that appears to be the agent name
                agentName = std::static_pointer_cast<Atomic>(option)->stringForm();
                continue;
            }
            // handle options with a key
            else if(option->termType() == TermType::FUNCTION) {
                // a key was specified for this option, ensure it is the "agent" ("a") key and use the value.
                auto fn = std::static_pointer_cast<Function>(option);
                if(fn->arity()==2) {
                    auto &key = fn->arguments()[0];
                    auto &value = fn->arguments()[1];
                    if(!agentName.has_value() && value->isAtom() &&
                            (*key == *a_agent1 || *key == *a_agent2))
                    {
                        agentName = std::static_pointer_cast<Atomic>(value)->stringForm();
                        continue;
                    }
                }
            }
            reportUnrecognized(option);
        }

        // create a parametrized modal operator
        if (agentName.has_value() && agentName.value() != "self") {
            return KnowledgeModality::K(agentName.value());
        }
    }
    return KnowledgeModality::K();
}

static ModalOperatorPtr createB(const TermPtr &optionsTerm)
{
	static const auto a_agent1 = Atom::Tabled("agent");
	static const auto a_agent2 = Atom::Tabled("a");
	static const auto a_confidence1 = Atom::Tabled("confidence");
	static const auto a_confidence2 = Atom::Tabled("c");

    if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
        auto listTerm = (ListTerm*)optionsTerm.get();
        std::optional<std::string> agentName;
        std::optional<double> confidenceValue;

        for(auto &option : *listTerm) {
            // handle options without key
            if(!agentName.has_value() && option->isAtom()) {
                // assume first string that appears to be the agent name
                agentName = std::static_pointer_cast<Atomic>(option)->stringForm();
                continue;
            }
            else if(!confidenceValue.has_value() && option->isNumeric()) {
                // assume first string that appears to be the agent name
                confidenceValue = std::static_pointer_cast<Numeric>(option)->asDouble();
                continue;
            }
            // handle options with a key
            else if(option->isFunction()) {
                // a key was specified for this option, ensure it is the "agent" ("a") key and use the value.
                auto fn = std::static_pointer_cast<Function>(option);
                if(fn->arity()==2) {
                    auto &key = fn->arguments()[0];
                    auto &value = fn->arguments()[1];
                    if(!agentName.has_value() && value->isAtom() &&
                            (*key == *a_agent1 || *key == *a_agent2))
                    {
                        agentName = std::static_pointer_cast<Atomic>(value)->stringForm();
                        continue;
                    }
                    else if(!confidenceValue.has_value() && value->isNumeric() &&
                            (*key == *a_confidence1 || *key == *a_confidence2))
                    {
                        confidenceValue = std::static_pointer_cast<Numeric>(value)->asDouble();
                        continue;
                    }
                }
            }
            reportUnrecognized(option);
        }

        if (agentName.has_value() && agentName.value() == "self") agentName = std::nullopt;
        // create a parametrized modal operator
        if (agentName.has_value()) {
            if(confidenceValue.has_value()) {
                return BeliefModality::B(agentName.value(), confidenceValue.value());
            }
            else {
                return BeliefModality::B(agentName.value());
            }
        }
        else if(confidenceValue.has_value()) {
            return BeliefModality::B(confidenceValue.value());
        }
    }
    return BeliefModality::B();
}

static inline std::optional<TimeInterval> readTimeInterval(ListTerm *options)
{
	static const auto a_begin = Atom::Tabled("begin");
	static const auto a_since = Atom::Tabled("since");
	static const auto a_end = Atom::Tabled("end");
	static const auto a_until = Atom::Tabled("until");

    std::optional<TimePoint> beginTime, endTime;

    for(auto &option : *options)
    {
        // TODO: allow eg `H[,20] phi`? i.e. empty for first arg?

		if(option->termType() == TermType::FUNCTION) {
        	// handle options with a key
            auto function = (Function*)option.get();
            if(function->arity()==2) {
                auto &key = function->arguments()[0];
                auto &value = function->arguments()[1];
                if(value->isNumeric()) {
                	auto numeric = std::static_pointer_cast<Numeric>(value);
					if(!beginTime.has_value() && (*key == *a_begin || *key == *a_since)) {
						beginTime = numeric->asDouble();
						continue;
					}
					else if(!endTime.has_value() && (*key == *a_end || *key == *a_until)) {
						endTime = numeric->asDouble();
						continue;
					}
				}
            }
        } else if (option->isNumeric()) {
        	// handle options without key
        	auto numeric = std::static_pointer_cast<Numeric>(option);
        	if(numeric) {
				if(!beginTime.has_value()) {
					beginTime = numeric->asDouble();
					continue;
				}
				else if(!endTime.has_value()) {
					endTime = numeric->asDouble();
					continue;
				}
        	}
        }
        reportUnrecognized(option);
    }

    if (beginTime.has_value() || endTime.has_value()) {
        return TimeInterval(beginTime, endTime);
    }
    else {
        return std::nullopt;
    }
}

static ModalOperatorPtr createP(const TermPtr &optionsTerm) {
    if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
        auto listTerm = (ListTerm*)optionsTerm.get();
        auto timeInterval = readTimeInterval(listTerm);
        if (timeInterval.has_value()) {
            return PastModality::P(timeInterval.value());
        }
    }
    return PastModality::P();
}

static ModalOperatorPtr createH(const TermPtr &optionsTerm) {
    if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
        auto listTerm = (ListTerm*)optionsTerm.get();
        auto timeInterval = readTimeInterval(listTerm);
        if (timeInterval.has_value()) {
            return PastModality::H(timeInterval.value());
        }
    }
    return PastModality::H();
}

static std::vector<TermPtr> createTermVector2(const TermPtr &a, const TermPtr &b) { return {a, b}; }

static AtomPtr makeAtom(std::string_view stringForm) { return Atom::Tabled(stringForm); }
static AtomPtr makeIRI(std::string_view stringForm) { return IRIAtom::Tabled(stringForm); }

QueryParser::QueryParser() {
    static const std::string equalFunctor = "=";

    // TODO: handle blanks here! Use Prolog convention that blank
    //       is indicated by a not enquoted word starting with an underscore.

    bnf_ = new ParserRules();
    bnf_->singleQuotes %= qi::lexeme['\'' >> +(qi::char_ - '\'') >> '\''];
    bnf_->doubleQuotes %= qi::lexeme['"' >> +(qi::char_ - '"') >> '"'];
    bnf_->lowerPrefix %= qi::raw[ascii::lower >> *(ascii::alnum | '_')];
    bnf_->upperPrefix %= qi::raw[ascii::upper >> *(ascii::alnum | '_')];

    // atoms are either a single quoted string, or words that start with a lowercase letter.
    bnf_->atomRaw %= (bnf_->singleQuotes | bnf_->lowerPrefix);
    bnf_->atom_regular = (bnf_->atomRaw
    		[qi::_val = boost::phoenix::bind(&makeAtom, qi::_1)]);

    // a IRI namespace is an alphanumeric word.
    // note that no single quotes are allowed, the greedy parser would match the `singleQuotes` rule before.
    bnf_->iri_ns %= (qi::raw[ascii::alpha >> *(ascii::alnum | '_')]);
    // right part of colon must be an alphanumeric word, or wrapped in single quoted.
    // Note that there is no need to enquote entities whose name starts with an uppercase character.
    bnf_->iri_entity %= (bnf_->singleQuotes | qi::raw[ascii::alpha >> *(ascii::alnum | '_')]);
    // IRI's are encoded as "ns:entity", ns must be a registered namespace at parse-time
    bnf_->iri_raw = ((bnf_->iri_ns >> qi::char_(':') >> bnf_->iri_entity)
            [qi::_val = boost::phoenix::bind(&createIRI, qi::_1, qi::_3)]);
	// TODO: support explicit typing of IRIs, e.g. iri('..')
	// TODO: parse string and guess if it is a IRI or not?
    bnf_->iri_atom = (bnf_->iri_raw
            [qi::_val = boost::phoenix::bind(&makeIRI, qi::_1)]);

    bnf_->atomRawWithIRI %= (bnf_->iri_raw | bnf_->atomRaw);

    ///////////////////////////
    // atomic constants: strings, numbers etc.
    bnf_->atom %= (bnf_->iri_atom | bnf_->atom_regular);
    bnf_->string = (bnf_->doubleQuotes
            [qi::_val = ptr_<String>()(qi::_1)]);
	// TODO: support explicit typing of numbers, e.g. `123::int`
    bnf_->number = (qi::double_
            [qi::_val = ptr_<Double>()(qi::_1)]);

    ///////////////////////////
    // option lists
    bnf_->keyvalue = ((((bnf_->atom) >> qi::char_('=')) >> (bnf_->atomic))
            [qi::_val = ptr_<Function>()(equalFunctor,
             boost::phoenix::bind(&createTermVector2, qi::_1, qi::_3))]);
    bnf_->option %= (bnf_->keyvalue | bnf_->atomic);
    bnf_->options = ((qi::char_('[') >> (bnf_->option % ',') >> qi::char_(']'))
            [qi::_val = ptr_<ListTerm>()(qi::_2)]);
    bnf_->nil = qi::attr(ListTerm::nil());

    ///////////////////////////
    // arguments of predicates: constants or variables
    bnf_->atomic %= (bnf_->atom | bnf_->string | bnf_->number);
    bnf_->list = ((qi::char_('[') >> (bnf_->atomic % ',') >> qi::char_(']'))
            [qi::_val = ptr_<ListTerm>()(qi::_2)]);
    bnf_->variable1 = (bnf_->upperPrefix[qi::_val = ptr_<Variable>()(qi::_1)]);
    bnf_->variable2 = (qi::char_('?') >> bnf_->lowerPrefix[qi::_val = ptr_<Variable>()(qi::_1)]);
    bnf_->variable %= (bnf_->variable1 | bnf_->variable2);
    // TODO: also support some operators like '<', '>' etc. without quotes
    // TODO: needs atom type
    bnf_->function = (((bnf_->atom) >>
            qi::char_('(') >> (bnf_->term % ',') >> ')')
            [qi::_val = ptr_<Function>()(qi::_1, qi::_3)]);
    bnf_->term %= bnf_->function | bnf_->variable | bnf_->atomic | bnf_->list;

    ///////////////////////////
    // predicates
    bnf_->predicateWithArgs = (((bnf_->atomRaw) >> qi::char_('(') >> (bnf_->term % ',') >> ')')
            [qi::_val = ptr_<Predicate>()(qi::_1, qi::_3)]);
    bnf_->predicateNullary = ((bnf_->atomRaw)
            [qi::_val = ptr_<Predicate>()(qi::_1, std::vector<TermPtr>())]);
    bnf_->predicate %= bnf_->predicateWithArgs | bnf_->predicateNullary;

    // formulas
    bnf_->brackets %= ('(' >> bnf_->formula >> ')');

    ///////////////////////////
    // unary operators
    bnf_->negation = (('~' >> (bnf_->unary | bnf_->brackets))
            [qi::_val = ~qi::_1]);
    bnf_->belief = (('B' >> (bnf_->options|bnf_->nil) >> (bnf_->unary | bnf_->brackets))
            [qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createB, qi::_1), qi::_2)]);
    bnf_->knowledge = (('K' >> (bnf_->options|bnf_->nil) >> (bnf_->unary | bnf_->brackets))
            [qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createK, qi::_1), qi::_2)]);
    bnf_->oncePast = (('P' >> (bnf_->options|bnf_->nil) >> (bnf_->unary | bnf_->brackets))
            [qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createP, qi::_1), qi::_2)]);
    bnf_->alwaysPast = (('H' >> (bnf_->options|bnf_->nil) >> (bnf_->unary | bnf_->brackets))
            [qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createH, qi::_1), qi::_2)]);

    bnf_->modalFormula %= bnf_->belief | bnf_->knowledge | bnf_->oncePast | bnf_->alwaysPast;
    bnf_->unary %= bnf_->modalFormula | bnf_->negation | bnf_->predicate;

    ///////////////////////////
    // compound formulae
    bnf_->conjunction = (((bnf_->unary | bnf_->brackets)
            >> (qi::char_(',') | qi::char_('&'))
            >> (bnf_->conjunction | bnf_->brackets))
                         [qi::_val = (qi::_1 & qi::_3)]
                         | bnf_->unary[qi::_val = qi::_1]);
    bnf_->disjunction = (((bnf_->conjunction | bnf_->brackets)
            >> (qi::char_(';') | qi::char_('|'))
            >> (bnf_->disjunction | bnf_->brackets))
                         [qi::_val = (qi::_1 | qi::_3)]
                         | bnf_->conjunction[qi::_val = qi::_1]);
    bnf_->implication = (((bnf_->disjunction | bnf_->brackets)
            >> "->"
            >> (bnf_->implication | bnf_->brackets))
                         [qi::_val = ptr_<Implication>()(qi::_1, qi::_2)]
                         | bnf_->disjunction[qi::_val = qi::_1]);

    bnf_->formula %= bnf_->implication | bnf_->brackets;
    //BOOST_SPIRIT_DEBUG_NODES((bnf_->conjunction)(bnf_->formula))
}

QueryParser::~QueryParser() {
    delete bnf_;
}

template<typename ResultType, typename RuleType>
static inline ResultType parse_(const std::string &queryString, const RuleType &rule) {
    auto first = queryString.begin();
    auto last = queryString.end();

    ResultType result;
    bool r = qi::phrase_parse(first, last, rule, ascii::space, result);

    if (first == last && r) {
        return result;
    } else {
        throw QueryError("Query string ({}) has invalid syntax.", queryString);
    }
}

ParserRules *QueryParser::get() {
    static QueryParser singleton;
    return singleton.bnf_;
}

FormulaPtr QueryParser::parse(const std::string &queryString) {
    return parse_<FormulaPtr, FormulaRule>(queryString, get()->formula);
}

PredicatePtr QueryParser::parsePredicate(const std::string &queryString) {
    return parse_<PredicatePtr, PredicateRule>(queryString, get()->predicate);
}

FunctionPtr QueryParser::parseFunction(const std::string &queryString) {
    return parse_<FunctionPtr, FunctionRule>(queryString, get()->function);
}

TermPtr QueryParser::parseConstant(const std::string &queryString) {
    return parse_<TermPtr, TermRule>(queryString, get()->atomic);
}

std::string QueryParser::parseRawAtom(const std::string &queryString) {
    return parse_<std::string, StringRule>(queryString, get()->atomRawWithIRI);
}

// fixture class for testing
namespace knowrob::testing {
	class QueryParserTest : public ::testing::Test {
	protected:
		// void SetUp() override { }
		// void TearDown() override {}
	};
}
using namespace knowrob::testing;

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
	GTEST_SKIP() << "syntax like P[,10.0] where an argument is missing is not supported yet by the query parser.";
    TEST_NO_THROW(testModal(QueryParser::parse("P[,10.0] p(x)"), "P", FormulaType::PREDICATE))
    TEST_NO_THROW(testModal(QueryParser::parse("P[10,] p(x)"), "P", FormulaType::PREDICATE))
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
