/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

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
        TermRule blank;
		TermRule xsd_term;

        AtomRule atom;
		AtomRule atom_regular;
        AtomRule iri_atom;


        StringRule singleQuotes;
        StringRule doubleQuotes;
        StringRule lowerPrefix;
        StringRule upperPrefix;
        StringRule rawBlank;
        StringRule atomRaw;
        StringRule atomRawWithIRI;
		StringRule number_string;
		StringRule xsd_value;

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
    bool nextIsBegin = true;

    for(auto &option : *options)
    {
		if(!option) {
			nextIsBegin = false;
			continue;
		} else if(option->termType() == TermType::FUNCTION) {
        	// handle options with a key
            auto function = (Function*)option.get();
            if(function->arity()==2) {
                auto &key = function->arguments()[0];
                auto &value = function->arguments()[1];
                if(value) {
                	if (value->isNumeric()) {
						auto numeric = std::static_pointer_cast<Numeric>(value);
						if((*key == *a_begin || *key == *a_since)) {
							beginTime = numeric->asDouble();
							continue;
						}
						else if((*key == *a_end || *key == *a_until)) {
							endTime = numeric->asDouble();
							continue;
						}
                	}
				}
            }
        } else if (option->isNumeric()) {
        	// handle options without key
        	auto numeric = std::static_pointer_cast<Numeric>(option);
			if(nextIsBegin) {
				beginTime = numeric->asDouble();
				nextIsBegin = false;
				continue;
			}
			else {
				endTime = numeric->asDouble();
				continue;
			}
        } else if (option->isAtomic() && std::static_pointer_cast<Atomic>(option)->stringForm() == "_") {
			nextIsBegin = false;
			continue;
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
static TermPtr makeXSD(std::string_view value, const AtomPtr &typeIRI) { return XSDAtomic::create(value, typeIRI->stringForm()); }

QueryParser::QueryParser() {
    static const std::string equalFunctor = "=";

    // TODO: handle blanks here! Use Prolog convention that blank
    //       is indicated by a not enquoted word starting with an underscore.

    bnf_ = new ParserRules();
    bnf_->singleQuotes %= qi::lexeme['\'' >> +(qi::char_ - '\'') >> '\''];
    bnf_->doubleQuotes %= qi::lexeme['"' >> +(qi::char_ - '"') >> '"'];
    bnf_->lowerPrefix %= qi::raw[ascii::lower >> *(ascii::alnum | '_')];
    bnf_->upperPrefix %= qi::raw[ascii::upper >> *(ascii::alnum | '_')];
    bnf_->rawBlank %= qi::raw['_'];
    bnf_->number_string %= qi::raw[qi::double_];

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
    // IRIs are encoded as "ns:entity", ns must be a registered namespace at parse-time
    bnf_->iri_raw = ((bnf_->iri_ns >> qi::char_(':') >> bnf_->iri_entity)
            [qi::_val = boost::phoenix::bind(&createIRI, qi::_1, qi::_3)]);
    bnf_->iri_atom = (bnf_->iri_raw
            [qi::_val = boost::phoenix::bind(&makeIRI, qi::_1)]);

    bnf_->atomRawWithIRI %= (bnf_->iri_raw | bnf_->atomRaw);

    ///////////////////////////
    // atomic constants: strings, numbers etc.
    bnf_->atom %= (bnf_->iri_atom | bnf_->atom_regular);
    bnf_->string = (bnf_->doubleQuotes
            [qi::_val = ptr_<String>()(qi::_1)]);
    bnf_->number = (qi::double_
            [qi::_val = ptr_<Double>()(qi::_1)]);
    bnf_->blank = (bnf_->rawBlank [qi::_val = ptr_<String>()(qi::_1)]);
    bnf_->xsd_value %= (bnf_->atomRaw | bnf_->number_string);
    bnf_->xsd_term = ((bnf_->xsd_value >> "^^" >> bnf_->iri_atom)
            [qi::_val = boost::phoenix::bind(&makeXSD, qi::_1, qi::_2)]);

    ///////////////////////////
    // option lists
    bnf_->keyvalue = (((bnf_->atom) >> qi::char_('=') >> (bnf_->atomic))
            [qi::_val = ptr_<Function>()(equalFunctor,
             boost::phoenix::bind(&createTermVector2, qi::_1, qi::_3))]);
    bnf_->option %= (bnf_->keyvalue | bnf_->atomic | bnf_->blank);
    bnf_->options = ((qi::char_('[') >> (bnf_->option % ',') >> qi::char_(']'))
            [qi::_val = ptr_<ListTerm>()(qi::_2)]);
    bnf_->nil = qi::attr(ListTerm::nil());

    ///////////////////////////
    // arguments of predicates: constants or variables
    bnf_->atomic %= (bnf_->atom | bnf_->string | bnf_->xsd_term | bnf_->number);
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
