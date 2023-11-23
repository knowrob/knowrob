#include <gtest/gtest.h>

#include "KnowledgeBaseTest.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

#define KB_TEST_SETTINGS_FILE "tests/settings/kb-test.json"

std::shared_ptr<knowrob::KnowledgeBase> KnowledgeBaseTest::kb_;

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

	AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal, int queryFlags) override {
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

	auto c1 = QueryParser::parseRawAtom("swrl_test:Ernest");
	kb_->reasonerManager()->addReasoner(
		"r1", std::make_shared<TestReasoner>("p", c1, "x"));
	kb_->reasonerManager()->addReasoner(
		"r2", std::make_shared<TestReasoner>("q", "x", "y"));
}

static std::vector<SubstitutionPtr> lookupAll(const std::string &queryString) {
	auto answerStream = KnowledgeBaseTest::kb_->submitQuery(
			QueryParser::parse(queryString), QUERY_FLAG_ALL_SOLUTIONS);
	auto answerQueue = answerStream->createQueue();
	std::vector<SubstitutionPtr> out;
	while(true) {
		auto solution = answerQueue->pop_front();
		if(AnswerStream::isEOS(solution)) break;
		out.push_back(solution->substitution());
	}
	return out;
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

TEST_F(KnowledgeBaseTest, undefinedNamespace) {
	EXPECT_THROW(lookupAll("undefined:hasSibling(swrl_test:Fred, X)"), QueryError);
}

TEST_F(KnowledgeBaseTest, atomic_EDB) {
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(swrl_test:Fred, X)").size(), 1);
	EXPECT_EQ(*lookupAll("swrl_test:hasSibling(swrl_test:Fred, X)")[0]->get("X"),
	          *QueryParser::parseConstant("swrl_test:Ernest"));
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(swrl_test:Lea, X)").size(), 0);
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(X, swrl_test:Ernest)").size(), 1);
	EXPECT_EQ(lookupAll("swrl_test:hasSibling(swrl_test:Fred, swrl_test:Ernest)").size(), 1);
}

TEST_F(KnowledgeBaseTest, modal_EDB) {
	EXPECT_EQ(lookupAll("B swrl_test:hasSibling(swrl_test:Fred, X)").size(), 1);
	EXPECT_EQ(*lookupAll("B swrl_test:hasSibling(swrl_test:Fred, X)")[0]->get("X"),
	          *QueryParser::parseConstant("swrl_test:Ernest"));
	EXPECT_EQ(lookupAll("K swrl_test:hasSibling(swrl_test:Fred, X)").size(), 1);
	EXPECT_EQ(*lookupAll("K swrl_test:hasSibling(swrl_test:Fred, X)")[0]->get("X"),
	          *QueryParser::parseConstant("swrl_test:Ernest"));
}

TEST_F(KnowledgeBaseTest, conjunctive_EDB) {
	const auto queryString = "swrl_test:hasSibling(swrl_test:Fred,X) , swrl_test:hasNumber(X,Num)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_EQ(*lookupAll(queryString)[0]->get("Num"), StringTerm("123456"));
}

TEST_F(KnowledgeBaseTest, disjunctive_EDB) {
	const auto queryString = "swrl_test:hasSibling(swrl_test:Fred,X) ; swrl_test:hasAncestor(swrl_test:Fred,X)";
	EXPECT_EQ(lookupAll(queryString).size(), 2);
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Ernest")));
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Rex")));
}

TEST_F(KnowledgeBaseTest, complex_EDB) {
	const auto queryString = "(swrl_test:hasSibling(swrl_test:Fred,X) ; "\
							 "swrl_test:hasAncestor(swrl_test:Fred,X)) , swrl_test:hasSibling(X,Y)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Ernest")));
}

TEST_F(KnowledgeBaseTest, negated_EDB) {
	const auto queryString = "~swrl_test:hasSibling(swrl_test:Lea,X)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_TRUE(lookupAll(queryString)[0]->empty());
}

TEST_F(KnowledgeBaseTest, negatedComplex_EDB) {
	const auto queryString = "(swrl_test:hasSibling(swrl_test:Fred,X) ; "\
		"swrl_test:hasAncestor(swrl_test:Fred,X)) , ~swrl_test:hasSibling(X,Y)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_TRUE(containsAnswer(lookupAll(queryString), "X", QueryParser::parseConstant("swrl_test:Rex")));
}

TEST_F(KnowledgeBaseTest, atomic_IDB) {
	EXPECT_EQ(lookupAll("p(swrl_test:Ernest, X)").size(), 1);
	EXPECT_EQ(*lookupAll("p(swrl_test:Ernest, X)")[0]->get("X"), StringTerm("x"));
	EXPECT_EQ(lookupAll("p(x, X)").size(), 0);
	EXPECT_EQ(lookupAll("q(x, X)").size(), 1);
}

TEST_F(KnowledgeBaseTest, mixed_EDB_IDB) {
	const auto queryString = "swrl_test:hasSibling(swrl_test:Fred,Ernest) , p(Ernest,Y)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_EQ(*lookupAll(queryString)[0]->get("Y"), StringTerm("x"));
}

TEST_F(KnowledgeBaseTest, IDB_interaction) {
	const auto queryString = "p(Ernest,X) , q(X,Y)";
	EXPECT_EQ(lookupAll(queryString).size(), 1);
	EXPECT_EQ(*lookupAll(queryString)[0]->get("Y"), StringTerm("y"));
}
