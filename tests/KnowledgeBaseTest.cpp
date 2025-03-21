#include <gtest/gtest.h>

#include "KnowledgeBaseTest.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/terms/String.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/semweb/GraphSequence.h"
#include "knowrob/integration/prolog/PrologBackend.h"
#include "knowrob/integration/prolog/PrologEngine.h"

using namespace knowrob;
using namespace knowrob::modals;

#define KB_TEST_SETTINGS_FILE "tests/settings/kb-test.json"

std::shared_ptr<knowrob::KnowledgeBase> KnowledgeBaseTest::kb_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::Fred_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::Ernest_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::Lea_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::Rex_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::hasSibling_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::hasNumber_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::hasAncestor_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::p_;
std::shared_ptr<knowrob::IRIAtom> KnowledgeBaseTest::q_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varX_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varY_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varZ_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varNum_;

// a reasoner that defines a single fact
class TestReasoner : public knowrob::GoalDrivenReasoner {
public:
	const std::string p_;
	const std::string s_;
	const std::string o_;
	TestReasoner(const std::string_view &p, const std::string_view &s, const std::string_view &o)
	: knowrob::GoalDrivenReasoner(), p_(p), s_(s), o_(o) {
		defineRelation(PredicateIndicator(p_, 2));
	}

	bool initializeReasoner(const PropertyTree& /*cfg*/) override { return true; }

	bool evaluate(GoalPtr query) override {
		auto &phi = query->formula();
		auto &literals = phi->literals();
		if(literals.size() != 1) {
			KB_ERROR("TestReasoner: expected a single literal");
			return false;
		}
		auto &literal = literals[0];
		auto &p = literal->predicate();
		EXPECT_EQ(p->arity(), 2);

		auto &subjectTerm = p->arguments()[0];
		auto &objectTerm = p->arguments()[1];
		bool succeed = true;
		if(subjectTerm->isGround()) {
			succeed = (*subjectTerm == IRIAtom(s_));
		}
		if(objectTerm->isGround()) {
			succeed = succeed && (*objectTerm == IRIAtom(o_));
		}

		if(succeed) {
			auto bindings = std::make_shared<Bindings>();
			if(!subjectTerm->isGround()) {
				auto v = *subjectTerm->variables().begin();
				bindings->set(std::make_shared<Variable>(v), IRIAtom::Tabled(s_));
			}
			if(!objectTerm->isGround()) {
				auto v = *objectTerm->variables().begin();
				bindings->set(std::make_shared<Variable>(v), IRIAtom::Tabled(o_));
			}
			auto answer = std::make_shared<AnswerYes>(bindings);
			query->push(answer);
		}

		return true;
	}
};

void KnowledgeBaseTest::SetUpTestSuite() {
	// initialize a KB, setup database backend for testing, insert tmp data on which queries can be evaluated
	kb_ = KnowledgeBase::create(KB_TEST_SETTINGS_FILE);

	Fred_   = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:Fred"));
	Ernest_ = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:Ernest"));
	Lea_    = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:Lea"));
	Rex_    = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:Rex"));
	hasSibling_  = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:hasSibling"));
	hasAncestor_  = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:hasAncestor"));
	hasNumber_  = IRIAtom::Tabled(QueryParser::parseRawAtom("swrl_test:hasNumber"));
	p_  = IRIAtom::Tabled(QueryParser::parseRawAtom("p"));
	q_  = IRIAtom::Tabled(QueryParser::parseRawAtom("q"));
	varX_ = std::make_shared<Variable>("X");
	varY_ = std::make_shared<Variable>("Y");
	varZ_ = std::make_shared<Variable>("Z");
	varNum_ = std::make_shared<Variable>("Num");

	kb_->reasonerManager()->addPlugin(
		"r1", PluginLanguage::CPP, std::make_shared<TestReasoner>("p", Ernest_->stringForm(), "x"));
	kb_->reasonerManager()->addPlugin(
		"r2", PluginLanguage::CPP, std::make_shared<TestReasoner>("q", "x", "y"));
}

void KnowledgeBaseTest::TearDownTestSuite() {
	kb_ = nullptr;
	Fred_ = nullptr;
	Ernest_ = nullptr;
	Lea_ = nullptr;
	Rex_ = nullptr;
	hasSibling_ = nullptr;
	hasAncestor_ = nullptr;
	hasNumber_ = nullptr;
	p_ = nullptr;
	q_ = nullptr;
	varX_ = nullptr;
	varY_ = nullptr;
	varZ_ = nullptr;
	varNum_ = nullptr;
}

static std::vector<BindingsPtr> lookup(const FormulaPtr &formula, const QueryContextPtr &ctx) {
	auto answerStream = KnowledgeBaseTest::kb_->submitQuery(formula, ctx);
	auto answerQueue = answerStream->createQueue();
	std::vector<BindingsPtr> out;
	while(true) {
		auto solution = answerQueue->pop_front();
		if(solution->indicatesEndOfEvaluation()) break;
		if(solution->tokenType() == TokenType::ANSWER_TOKEN) {
			auto answer = std::static_pointer_cast<const Answer>(solution);
			if(answer->isPositive()) {
				auto answerYes = std::static_pointer_cast<const AnswerYes>(answer);
				out.push_back(answerYes->substitution());
			}
		}
	}
	return out;
}

static std::vector<BindingsPtr> lookupAll(const std::string &queryString) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
    return lookup(QueryParser::parse(queryString), ctx);
}

static std::vector<BindingsPtr> lookupAll(const FormulaPtr &p) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
    return lookup(p, ctx);
}

static bool containsAnswer(const std::vector<BindingsPtr> &answers, const std::string &key, const TermPtr &value) {
	for (auto &x: answers) {
		if (x->contains(key)) {
			auto actual = x->get(key);
			if (*value == *actual) {
				return true;
			}
		}
	}

	return false;
}

#define EXPECT_ONLY_SOLUTION(phi, sol) { \
	auto sols = lookupAll(phi);              \
	EXPECT_EQ(sols.size(),1);               \
	if(sols.size()==1) { EXPECT_EQ(*sols[0], sol); } }

#define EXPECT_NO_SOLUTION(phi) EXPECT_EQ(lookupAll(phi).size(),0)

TEST_F(KnowledgeBaseTest, undefinedNamespace) {
	EXPECT_THROW(lookupAll("undefined:hasSibling(swrl_test:Fred, X)"), QueryError);
}

TEST_F(KnowledgeBaseTest, observe_predicate) {
	auto observedPredicate = (*hasAncestor_)(varX_, varY_);
	auto term = std::make_shared<GraphPattern>(
			std::make_shared<TriplePattern>(observedPredicate));
	auto query = std::make_shared<GraphQuery>(term);
	uint32_t counter = 0;
	auto observer = kb_->observe(query, [&counter](const BindingsPtr&) {
		counter += 1;
	});

	TripleCopy newTriple;
	newTriple.setSubject(Lea_->stringForm());
	newTriple.setPredicate(hasAncestor_->stringForm());
	newTriple.setObjectIRI(Ernest_->stringForm());

	uint32_t expectedCounter = counter + 1;
	kb_->insertOne(newTriple);
	kb_->synchronizeObservers();
	EXPECT_EQ(counter, expectedCounter);
}

TEST_F(KnowledgeBaseTest, observe_sequence) {
	auto observedPredicate1 = (*hasAncestor_)(varX_, varY_);
	auto observedPredicate2 = (*hasAncestor_)(varY_, varZ_);
	std::shared_ptr<GraphTerm> term1 = std::make_shared<GraphPattern>(
			std::make_shared<TriplePattern>(observedPredicate1));
	std::shared_ptr<GraphTerm> term2 = std::make_shared<GraphPattern>(
			std::make_shared<TriplePattern>(observedPredicate2));
	auto seq = std::make_shared<GraphSequence>(std::vector<std::shared_ptr<GraphTerm>>{term1, term2});
	auto query = std::make_shared<GraphQuery>(seq);
	uint32_t counter = 0;
	auto observer = kb_->observe(query, [&counter](const BindingsPtr&) {
		counter += 1;
	});

	TripleCopy newTriple;
	newTriple.setSubject(Lea_->stringForm());
	newTriple.setPredicate(hasAncestor_->stringForm());
	newTriple.setObjectIRI(Ernest_->stringForm());

	uint32_t expectedCounter = counter;
	kb_->insertOne(newTriple);
	kb_->synchronizeObservers();
	EXPECT_EQ(counter, expectedCounter);

	expectedCounter += 1;
	newTriple.setSubject(Ernest_->stringForm());
	newTriple.setObjectIRI(Rex_->stringForm());
	kb_->insertOne(newTriple);
	kb_->synchronizeObservers();
	EXPECT_EQ(counter, expectedCounter);
}

TEST_F(KnowledgeBaseTest, atomic_EDB) {
	EXPECT_ONLY_SOLUTION(
		// the query formula:
		(*hasSibling_)(Fred_, varX_),
		// the expected solution as substitution mapping:
		Bindings({{varX_, Ernest_}}))
	EXPECT_ONLY_SOLUTION(
			(*hasSibling_)(varX_, Ernest_),
			Bindings({{varX_, Fred_}}))
	EXPECT_ONLY_SOLUTION(
			(*hasSibling_)(Fred_, Ernest_),
			Bindings())
	// negative case without any solution:
	EXPECT_NO_SOLUTION((*hasSibling_)(Lea_, varX_));
}

TEST_F(KnowledgeBaseTest, conjunctive_EDB) {
	EXPECT_ONLY_SOLUTION(
	(*hasSibling_)(Fred_, varX_) & (*hasNumber_)(varX_, varNum_),
	Bindings({
			{varNum_, std::make_shared<String>("123456")},
			{varX_, Ernest_}
		}))
}

TEST_F(KnowledgeBaseTest, disjunctive_EDB) {
	auto sols = lookupAll(
		(*hasSibling_)(Fred_, varX_) |
		(*hasAncestor_)(Fred_, varX_));
	EXPECT_EQ(sols.size(), 2);
	EXPECT_TRUE(containsAnswer(sols, "X", Ernest_));
	EXPECT_TRUE(containsAnswer(sols, "X", Rex_));
}

TEST_F(KnowledgeBaseTest, complex_EDB) {
	EXPECT_ONLY_SOLUTION(
	((*hasSibling_)(Fred_, varX_) | (*hasAncestor_)(Fred_, varX_)) & (*hasSibling_)(varX_, varY_),
	Bindings({
			{varX_, Ernest_},
			{varY_, Fred_}
		}))
}

TEST_F(KnowledgeBaseTest, negated_EDB) {
	EXPECT_ONLY_SOLUTION(
			~(*hasSibling_)(Lea_, varX_),
			Bindings())
	EXPECT_NO_SOLUTION(~(*hasSibling_)(Fred_, varX_));
}

TEST_F(KnowledgeBaseTest, negated_IDB) {
	EXPECT_ONLY_SOLUTION(
			~(*p_)(Lea_, varX_),
			Bindings())
	EXPECT_NO_SOLUTION(~(*p_)(Ernest_, varX_));
}

TEST_F(KnowledgeBaseTest, negatedComplex_EDB) {
	// Rex is an ancestor of Fred who does not have a sibling
	EXPECT_ONLY_SOLUTION(
		(*hasAncestor_)(Fred_, varX_) & ~(*hasSibling_)(varX_, varY_),
		Bindings({{varX_, Rex_}}))
}

TEST_F(KnowledgeBaseTest, atomic_IDB) {
	EXPECT_ONLY_SOLUTION(
			(*p_)(Ernest_, varX_),
			Bindings({{varX_, IRIAtom::Tabled("x")}}))
	EXPECT_ONLY_SOLUTION(
			(*q_)(IRIAtom::Tabled("x"), varX_),
			Bindings({{varX_, IRIAtom::Tabled("y")}}))
	EXPECT_NO_SOLUTION((*p_)(IRIAtom::Tabled("x"), varX_));
}

TEST_F(KnowledgeBaseTest, mixed_EDB_IDB) {
	EXPECT_ONLY_SOLUTION(
	(*hasSibling_)(Fred_, varX_) & (*p_)(varX_, varY_),
	Bindings({
			{varX_, Ernest_},
			{varY_, IRIAtom::Tabled("x")}
		}))
}

TEST_F(KnowledgeBaseTest, IDB_interaction) {
	EXPECT_ONLY_SOLUTION(
	(*p_)(varX_, varY_) & (*q_)(varY_, varZ_),
	Bindings({
			{varX_, Ernest_},
			{varY_, IRIAtom::Tabled("x")},
			{varZ_, IRIAtom::Tabled("y")}
		}))
}

TEST_F(KnowledgeBaseTest, modal_EDB_K) {
	EXPECT_ONLY_SOLUTION(
			K((*hasSibling_)(Fred_, varX_)),
			Bindings({{varX_, Ernest_}}))
}

TEST_F(KnowledgeBaseTest, modal_EDB_B) {
	EXPECT_ONLY_SOLUTION(
			B((*hasSibling_)(Fred_, varX_)),
			Bindings({{varX_, Ernest_}}))
}

TEST_F(KnowledgeBaseTest, esg_json) {
	kb_ = KnowledgeBase::create("tests/settings/esg.json");
	EXPECT_ONLY_SOLUTION(
			"soma:during(events:Event5, X)",
			Bindings({{varX_, iri("events","Event4")}}))
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, lpn_json) {
	if (PrologEngine::isPrologInitialized()) {
		PrologBackend::removeAll();
	}
	kb_ = KnowledgeBase::create("tests/settings/lpn.json");
	EXPECT_ONLY_SOLUTION(
			"lpn:jealous(lpn:vincent, X)",
			Bindings({{varX_, iri("lpn","marsellus")}}))
	if (PrologEngine::isPrologInitialized()) {
		PrologBackend::removeAll();
	}
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, lpn_py) {
	kb_ = KnowledgeBase::create("tests/plugins/lpn-py.json");
	EXPECT_ONLY_SOLUTION(
			"lpn:jealous(lpn:vincent, X)",
			Bindings({{varX_, iri("lpn","marsellus")}}))
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, lpn_c) {
	kb_ = KnowledgeBase::create("tests/plugins/lpn-c.json");
	EXPECT_ONLY_SOLUTION(
			"lpn:jealous(lpn:vincent, X)",
			Bindings({{varX_, iri("lpn","marsellus")}}))
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, lpn_c_list_term) {
	kb_ = KnowledgeBase::create("tests/plugins/lpn-c.json");
	auto zero = XSDAtomic::create("0.0", xsdTypeToIRI(XSDType::DOUBLE));
	auto posVal = std::make_shared<ListTerm>(std::vector<TermPtr>{zero, zero, zero});
	auto quatVal = std::make_shared<ListTerm>(std::vector<TermPtr>{zero, zero, zero, zero});
	EXPECT_ONLY_SOLUTION(
			"lpn:pos(foo, X)",
			Bindings({{varX_, posVal}}))
	EXPECT_ONLY_SOLUTION(
			"lpn:quaternion(foo, X)",
			Bindings({{varX_, quatVal}}))
	EXPECT_ONLY_SOLUTION(
			"lpn:pos(foo, X) & lpn:quaternion(foo, Y)",
			Bindings({{varX_, posVal}, {varY_, quatVal}}))
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, mongolog_lpn_json) {
	kb_ = KnowledgeBase::create("tests/settings/mongolog-lpn.json");
	EXPECT_ONLY_SOLUTION(
			"lpn:jealous(lpn:vincent, X)",
			Bindings({{varX_, iri("lpn", "marsellus")}}))
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, swrl_json) {
	if (PrologEngine::isPrologInitialized()) {
		PrologBackend::removeAll();
	}
	kb_ = KnowledgeBase::create("tests/settings/swrl.json");
	EXPECT_ONLY_SOLUTION(
			"swrl:Hermaphrodite(X)",
			Bindings({{varX_, iri("swrl","Lea")}}))
	if (PrologEngine::isPrologInitialized()) {
		PrologBackend::removeAll();
	}
	kb_ = nullptr;
}

TEST_F(KnowledgeBaseTest, srdl_json) {
	if (PrologEngine::isPrologInitialized()) {
		PrologBackend::removeAll();
	}
	kb_ = KnowledgeBase::create("tests/settings/srdl.json");
	EXPECT_ONLY_SOLUTION(
			"srdl2cap:hasCapability(pr2:PR2_0, srdl2cap:NavigationCapability)",
			Bindings())
	if (PrologEngine::isPrologInitialized()) {
		PrologBackend::removeAll();
	}
	kb_ = nullptr;
}
