/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/spirit/include/phoenix.hpp>
#include "knowrob/queries/parsers/common.h"
#include "knowrob/queries/parsers/formula.h"
#include "knowrob/queries/parsers/terms.h"
#include "knowrob/queries/parsers/strings.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Implication.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/modalities/KnowledgeModality.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/modalities/TimeInterval.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/PastModality.h"

#define RETURN_FORMULA_RULE(expr) static FormulaRule r(expr); return r
#define RETURN_PREDICATE_RULE(expr) static PredicateRule r(expr); return r

using namespace knowrob;

static void reportUnrecognized(const TermPtr &option) {
	throw QueryError("Unrecognized option ({}) in modal operator.", *option);
}

static ModalOperatorPtr createK(const TermPtr &optionsTerm) {
	static const auto a_agent1 = Atom::Tabled("agent");
	static const auto a_agent2 = Atom::Tabled("a");

	if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
		auto listTerm = (ListTerm *) optionsTerm.get();
		std::optional<std::string> agentName;

		for (auto &option: *listTerm) {
			// handle options without key
			if (!agentName.has_value() && option->isAtom()) {
				// assume first string that appears to be the agent name
				agentName = std::static_pointer_cast<Atomic>(option)->stringForm();
				continue;
			}
				// handle options with a key
			else if (option->termType() == TermType::FUNCTION) {
				// a key was specified for this option, ensure it is the "agent" ("a") key and use the value.
				auto fn = std::static_pointer_cast<Function>(option);
				if (fn->arity() == 2) {
					auto &key = fn->arguments()[0];
					auto &value = fn->arguments()[1];
					if (!agentName.has_value() && value->isAtom() &&
						(*key == *a_agent1 || *key == *a_agent2)) {
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

static ModalOperatorPtr createB(const TermPtr &optionsTerm) {
	static const auto a_agent1 = Atom::Tabled("agent");
	static const auto a_agent2 = Atom::Tabled("a");
	static const auto a_confidence1 = Atom::Tabled("confidence");
	static const auto a_confidence2 = Atom::Tabled("c");

	if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
		auto listTerm = (ListTerm *) optionsTerm.get();
		std::optional<std::string> agentName;
		std::optional<double> confidenceValue;

		for (auto &option: *listTerm) {
			// handle options without key
			if (!agentName.has_value() && option->isAtom()) {
				// assume first string that appears to be the agent name
				agentName = std::static_pointer_cast<Atomic>(option)->stringForm();
				continue;
			} else if (!confidenceValue.has_value() && option->isNumeric()) {
				// assume first string that appears to be the agent name
				confidenceValue = std::static_pointer_cast<Numeric>(option)->asDouble();
				continue;
			}
				// handle options with a key
			else if (option->isFunction()) {
				// a key was specified for this option, ensure it is the "agent" ("a") key and use the value.
				auto fn = std::static_pointer_cast<Function>(option);
				if (fn->arity() == 2) {
					auto &key = fn->arguments()[0];
					auto &value = fn->arguments()[1];
					if (!agentName.has_value() && value->isAtom() &&
						(*key == *a_agent1 || *key == *a_agent2)) {
						agentName = std::static_pointer_cast<Atomic>(value)->stringForm();
						continue;
					} else if (!confidenceValue.has_value() && value->isNumeric() &&
							   (*key == *a_confidence1 || *key == *a_confidence2)) {
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
			if (confidenceValue.has_value()) {
				return BeliefModality::B(agentName.value(), confidenceValue.value());
			} else {
				return BeliefModality::B(agentName.value());
			}
		} else if (confidenceValue.has_value()) {
			return BeliefModality::B(confidenceValue.value());
		}
	}
	return BeliefModality::B();
}

static inline std::optional<TimeInterval> readTimeInterval(ListTerm *options) {
	static const auto a_begin = Atom::Tabled("begin");
	static const auto a_since = Atom::Tabled("since");
	static const auto a_end = Atom::Tabled("end");
	static const auto a_until = Atom::Tabled("until");

	std::optional<TimePoint> beginTime, endTime;
	bool nextIsBegin = true;

	for (auto &option: *options) {
		if (!option) {
			nextIsBegin = false;
			continue;
		} else if (option->termType() == TermType::FUNCTION) {
			// handle options with a key
			auto function = (Function *) option.get();
			if (function->arity() == 2) {
				auto &key = function->arguments()[0];
				auto &value = function->arguments()[1];
				if (value) {
					if (value->isNumeric()) {
						auto numeric = std::static_pointer_cast<Numeric>(value);
						if ((*key == *a_begin || *key == *a_since)) {
							beginTime = time::fromSeconds(numeric->asDouble());
							continue;
						} else if ((*key == *a_end || *key == *a_until)) {
							endTime = time::fromSeconds(numeric->asDouble());
							continue;
						}
					}
				}
			}
		} else if (option->isNumeric()) {
			// handle options without key
			auto numeric = std::static_pointer_cast<Numeric>(option);
			if (nextIsBegin) {
				beginTime = time::fromSeconds(numeric->asDouble());
				nextIsBegin = false;
				continue;
			} else {
				endTime = time::fromSeconds(numeric->asDouble());
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
	} else {
		return std::nullopt;
	}
}

static ModalOperatorPtr createP(const TermPtr &optionsTerm) {
	if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
		auto listTerm = (ListTerm *) optionsTerm.get();
		auto timeInterval = readTimeInterval(listTerm);
		if (timeInterval.has_value()) {
			return PastModality::P(timeInterval.value());
		}
	}
	return PastModality::P();
}

static ModalOperatorPtr createH(const TermPtr &optionsTerm) {
	if (optionsTerm && optionsTerm.get() != ListTerm::nil().get()) {
		auto listTerm = (ListTerm *) optionsTerm.get();
		auto timeInterval = readTimeInterval(listTerm);
		if (timeInterval.has_value()) {
			return PastModality::H(timeInterval.value());
		}
	}
	return PastModality::H();
}

namespace knowrob::parsers::formula {
	using namespace knowrob::parsers::terms;
	namespace qi = boost::spirit::qi;

	PredicateRule &predicate_n() {
		RETURN_PREDICATE_RULE((str::atom() >> '(' >> (term() % ',') >> ')')
		                      [qi::_val = ptr_<Predicate>()(qi::_1, qi::_2)]);
	}

	PredicateRule &predicate_0() {
		RETURN_PREDICATE_RULE(str::atom() [qi::_val = ptr_<Predicate>()(qi::_1, std::vector<TermPtr>())]);
	}

	PredicateRule &predicate() {
		RETURN_PREDICATE_RULE(predicate_n() | predicate_0());
	}

	struct parsers_struct {
		parsers_struct() {
			formula %= implication | brackets;
			brackets %= ('(' >>formula >> ')');

			implication = (((disjunction | brackets)
				>> "->"
				>> (implication | brackets))
								[qi::_val = ptr_<Implication>()(qi::_1, qi::_2)]
								| disjunction[qi::_val = qi::_1]
			);
			disjunction = (((conjunction | brackets)
					>> (qi::char_(';') | qi::char_('|'))
					>> (disjunction | brackets))
								 [qi::_val = (qi::_1 | qi::_3)]
								 | conjunction[qi::_val = qi::_1]);
			conjunction = (((unary | brackets)
					>> (qi::char_(',') | qi::char_('&'))
					>> (conjunction | brackets))
								 [qi::_val = (qi::_1 & qi::_3)]
								 | unary[qi::_val = qi::_1]);

			unary %= modal | negation | predicate();
			negation = (('~' >> (unary | brackets)) [qi::_val = ~qi::_1]);
			modal %= belief | knowledge | occasional | always;

			belief = (
				('B' >> options_or_nil() >> (unary | brackets))
				[qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createB, qi::_1), qi::_2)]
			);
			knowledge = (
				('K' >> options_or_nil() >> (unary | brackets))
				[qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createK, qi::_1), qi::_2)]
			);
			occasional = (
				('P' >> options_or_nil() >> (unary | brackets))
				[qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createP, qi::_1), qi::_2)]
			);
			always = (
				('H' >> options_or_nil() >> (unary | brackets))
				[qi::_val = ptr_<ModalFormula>()(boost::phoenix::bind(&createH, qi::_1), qi::_2)]
			);
		}

		FormulaRule formula;
		FormulaRule brackets;
		FormulaRule implication;
		FormulaRule disjunction, conjunction;
		FormulaRule unary;
		FormulaRule negation;
		FormulaRule modal, belief, knowledge, occasional, always;
	};

	auto &parsers() {
		static parsers_struct p;
		return p;
	}

	FormulaRule &formula() {
		return parsers().formula;
	}
}
