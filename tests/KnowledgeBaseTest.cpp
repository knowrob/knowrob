#include <gtest/gtest.h>

#include "KnowledgeBaseTest.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/formulas/ModalFormula.h"

using namespace knowrob;
using namespace knowrob::modality;

#define KB_TEST_SETTINGS_FILE "tests/settings/kb-test.json"

std::shared_ptr<knowrob::KnowledgeBase> KnowledgeBaseTest::kb_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::Fred_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::Ernest_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::Lea_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::Rex_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::hasSibling_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::hasNumber_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::hasAncestor_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::p_;
std::shared_ptr<knowrob::StringTerm> KnowledgeBaseTest::q_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varX_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varY_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varZ_;
std::shared_ptr<knowrob::Variable> KnowledgeBaseTest::varNum_;

// a reasoner that defines a single fact
class TestReasoner : public knowrob::Reasoner {
public:
	const std::string p_;
	const std::string s_;
	const std::string o_;
	TestReasoner(const std::string_view &p, const std::string_view &s, const std::string_view &o)
	: knowrob::Reasoner(), p_(p), s_(s), o_(o) {}

	bool loadConfiguration(const ReasonerConfiguration &cfg) override { return true; }
	void setDataBackend(const KnowledgeGraphPtr &knowledgeGraph) override {}
	unsigned long getCapabilities() const override { return CAPABILITY_TOP_DOWN_EVALUATION; };

	std::shared_ptr<PredicateDescription> getPredicateDescription(
		const std::shared_ptr<PredicateIndicator> &indicator) override {
		if(indicator->functor() == p_) {
			return std::make_shared<PredicateDescription>(
				std::make_shared<PredicateIndicator>(p_,2), PredicateType::RELATION);
		}
		else {
			return {};
		}
	}

	AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx) override {
		auto answerBuffer = std::make_shared<AnswerBuffer>();
		auto outputChannel = AnswerStream::Channel::create(answerBuffer);
		auto answer = std::make_shared<Answer>();

		bool succeed = true;
		if(literal->propertyTerm()->isGround()) {
			succeed = (*literal->propertyTerm() == StringTerm(p_));
		}
		if(literal->subjectTerm()->isGround()) {
			succeed = succeed && (*literal->subjectTerm() == StringTerm(s_));
		}
		if(literal->objectTerm()->isGround()) {
			succeed = succeed && (*literal->objectTerm() == StringTerm(o_));
		}

		if(succeed) {
			if(!literal->propertyTerm()->isGround()) {
				auto v = *literal->propertyTerm()->getVariables().begin();
				answer->substitution()->set(*v, std::make_shared<StringTerm>(p_));
			}
			if(!literal->subjectTerm()->isGround()) {
				auto v = *literal->subjectTerm()->getVariables().begin();
				answer->substitution()->set(*v, std::make_shared<StringTerm>(s_));
			}
			if(!literal->objectTerm()->isGround()) {
				auto v = *literal->objectTerm()->getVariables().begin();
				answer->substitution()->set(*v, std::make_shared<StringTerm>(o_));
			}
			outputChannel->push(answer);
		}

		outputChannel->push(AnswerStream::eos());
		return answerBuffer;
	}
};

void KnowledgeBaseTest::SetUpTestSuite() {
	// initialize a KB, setup database backend for testing, insert tmp data on which queries can be evaluated
	kb_ = std::make_shared<KnowledgeBase>(KB_TEST_SETTINGS_FILE);

	Fred_   = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:Fred"));
	Ernest_ = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:Ernest"));
	Lea_    = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:Lea"));
	Rex_    = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:Rex"));
	hasSibling_  = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:hasSibling"));
	hasAncestor_  = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:hasAncestor"));
	hasNumber_  = std::make_shared<StringTerm>(QueryParser::parseRawAtom("swrl_test:hasNumber"));
	p_  = std::make_shared<StringTerm>(QueryParser::parseRawAtom("p"));
	q_  = std::make_shared<StringTerm>(QueryParser::parseRawAtom("q"));
	varX_ = std::make_shared<Variable>("X");
	varY_ = std::make_shared<Variable>("Y");
	varZ_ = std::make_shared<Variable>("Z");
	varNum_ = std::make_shared<Variable>("Num");

	kb_->reasonerManager()->addReasoner(
		"r1", std::make_shared<TestReasoner>("p", Ernest_->value(), "x"));
	kb_->reasonerManager()->addReasoner(
		"r2", std::make_shared<TestReasoner>("q", "x", "y"));
}

static std::vector<SubstitutionPtr> lookup(const FormulaPtr &formula, const QueryContextPtr &ctx) {
	auto answerStream = KnowledgeBaseTest::kb_->submitQuery(formula, ctx);
	auto answerQueue = answerStream->createQueue();
	std::vector<SubstitutionPtr> out;
	while(true) {
		auto solution = answerQueue->pop_front();
		if(AnswerStream::isEOS(solution)) break;
		out.push_back(solution->substitution());
	}
	return out;
}

static std::vector<SubstitutionPtr> lookupAll(const std::string &queryString) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
    return lookup(QueryParser::parse(queryString), ctx);
}

static std::vector<SubstitutionPtr> lookupAll(const FormulaPtr &p) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
    return lookup(p, ctx);
}

static std::vector<SubstitutionPtr> lookupOne(const std::string &queryString) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ONE_SOLUTION);
    return lookup(QueryParser::parse(queryString), ctx);
}

static SubstitutionPtr lookupOne(const FormulaPtr &p) {
	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ONE_SOLUTION);
    auto substitutions = lookup(p, ctx);
    if(substitutions.empty()) {
    	return {};
    }
    else {
    	return substitutions[0];
    }
}

static bool containsAnswer(const std::vector<SubstitutionPtr> &answers, const std::string &key, const TermPtr &value) {
	Variable v_key(key);
	for (auto &x: answers) {
		if (x->contains(v_key)) {
			auto actual = x->get(v_key);
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
	if(sols.size()==1) EXPECT_EQ(*sols[0], sol); }

#define EXPECT_NO_SOLUTION(phi) EXPECT_EQ(lookupAll(phi).size(),0)

TEST_F(KnowledgeBaseTest, undefinedNamespace) {
	EXPECT_THROW(lookupAll("undefined:hasSibling(swrl_test:Fred, X)"), QueryError);
}

TEST_F(KnowledgeBaseTest, atomic_EDB) {
	EXPECT_ONLY_SOLUTION(
		// the query formula:
		(*hasSibling_)(Fred_, varX_),
		// the expected solution as substitution mapping:
		Substitution({{*varX_, Ernest_}}));
	EXPECT_ONLY_SOLUTION(
		(*hasSibling_)(varX_, Ernest_),
		Substitution({{*varX_, Fred_}}));
	EXPECT_ONLY_SOLUTION(
		(*hasSibling_)(Fred_, Ernest_),
		Substitution());
	// negative case without any solution:
	EXPECT_NO_SOLUTION((*hasSibling_)(Lea_, varX_));
}

TEST_F(KnowledgeBaseTest, conjunctive_EDB) {
	EXPECT_ONLY_SOLUTION(
		(*hasSibling_)(Fred_, varX_) & (*hasNumber_)(varX_, varNum_),
		Substitution({
			{*varNum_, std::make_shared<StringTerm>("123456")},
			{*varX_, Ernest_}
		}));
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
		Substitution({
			{*varX_, Ernest_},
			{*varY_, Fred_}
		}));
}

TEST_F(KnowledgeBaseTest, negated_EDB) {
	EXPECT_ONLY_SOLUTION(
		~(*hasSibling_)(Lea_, varX_),
		Substitution());
	EXPECT_NO_SOLUTION(~(*hasSibling_)(Fred_, varX_));
}

TEST_F(KnowledgeBaseTest, negated_IDB) {
	EXPECT_ONLY_SOLUTION(
		~(*p_)(Lea_, varX_),
		Substitution());
	EXPECT_NO_SOLUTION(~(*p_)(Ernest_, varX_));
}

TEST_F(KnowledgeBaseTest, negatedComplex_EDB) {
	// Rex is an ancestor of Fred who does not have a sibling
	EXPECT_ONLY_SOLUTION(
		(*hasAncestor_)(Fred_, varX_) & ~(*hasSibling_)(varX_, varY_),
		Substitution({{*varX_, Rex_}}));
}

TEST_F(KnowledgeBaseTest, atomic_IDB) {
	EXPECT_ONLY_SOLUTION(
		(*p_)(Ernest_, varX_),
		Substitution({{*varX_, std::make_shared<StringTerm>("x")}}));
	EXPECT_ONLY_SOLUTION(
		(*q_)(std::make_shared<StringTerm>("x"), varX_),
		Substitution({{*varX_, std::make_shared<StringTerm>("y")}}));
	EXPECT_NO_SOLUTION((*p_)(std::make_shared<StringTerm>("x"), varX_));
}

TEST_F(KnowledgeBaseTest, mixed_EDB_IDB) {
	EXPECT_ONLY_SOLUTION(
		(*hasSibling_)(Fred_, varX_) & (*p_)(varX_, varY_),
		Substitution({
			{*varX_, Ernest_},
			{*varY_, std::make_shared<StringTerm>("x")}
		}));
}

TEST_F(KnowledgeBaseTest, IDB_interaction) {
	EXPECT_ONLY_SOLUTION(
		(*p_)(varX_, varY_) & (*q_)(varY_, varZ_),
		Substitution({
			{*varX_, Ernest_},
			{*varY_, std::make_shared<StringTerm>("x")},
			{*varZ_, std::make_shared<StringTerm>("y")}
		}));
}

TEST_F(KnowledgeBaseTest, modal_EDB) {
	EXPECT_ONLY_SOLUTION(
		K((*hasSibling_)(Fred_, varX_)),
		Substitution({{*varX_, Ernest_}}));
	EXPECT_ONLY_SOLUTION(
		B((*hasSibling_)(Fred_, varX_)),
		Substitution({{*varX_, Ernest_}}));
}
