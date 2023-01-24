
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/queries.h>
#include <knowrob/Blackboard.h>
#include <knowrob/reasoner.h>
#include <knowrob/prolog/PrologQuery.h>
#include <knowrob/prolog/PrologReasoner.h>
#include <knowrob/mongolog/MongologReasoner.h>
#include <knowrob/knowrob.h>

//using namespace knowrob;
void test_kb1_PrologReasoner(
	const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
	const std::shared_ptr<knowrob::ReasonerManager> &reasonerManager)
{
	auto query1 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X)"));
	auto query2 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X), loves(Y,X)"));
	auto query3 = knowrob::PrologQuery::toQuery(reasoner->readTerm("abcdef(X)"));
	auto query4 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X), robot(Y)"));

	KB_INFO("PrologReasoner query: {}", (*query1.get()));
	for(auto &solution : reasoner->allSolutions(query1))
	{
		KB_INFO("{}", *solution->substitution());
	}

	KB_INFO("PrologReasoner query: {}", (*query2.get()));
	for(auto &solution : reasoner->allSolutions(query2))
	{
		KB_INFO("{}", *solution->substitution());
	}

	KB_INFO("PrologReasoner query: {}", (*query3.get()));
	for(auto &solution : reasoner->allSolutions(query3))
	{
		KB_INFO("{}", *solution->substitution());
	}

	KB_INFO("Blackboard query: {}", (*query1.get())); {
		auto bbq = std::make_shared<knowrob::QueryResultQueue>();
		auto bb = std::make_shared<knowrob::Blackboard>(reasonerManager, bbq, query1);
		bb->start();
		while(true) {
			auto solution = bbq->pop_front();
			if(knowrob::QueryResultStream::isEOS(solution)) break;
			KB_INFO("{}", *solution->substitution());
		}
	}

	KB_INFO("Blackboard query: {}", (*query3.get())); {
		auto bbq = std::make_shared<knowrob::QueryResultQueue>();
		try {
			auto bb = std::make_shared<knowrob::Blackboard>(reasonerManager, bbq, query3);
		}
		catch(const knowrob::QueryError& e) {
			KB_WARN("query error: {}", e.what());
		}
	}

	KB_INFO("Blackboard query: {}", (*query4.get())); {
		auto bbq = std::make_shared<knowrob::QueryResultQueue>();
		auto bb = std::make_shared<knowrob::Blackboard>(reasonerManager, bbq, query4);
		bb->start();
		while(true) {
			auto solution = bbq->pop_front();
			if(knowrob::QueryResultStream::isEOS(solution)) break;
			KB_INFO("{}", *solution->substitution());
		}
	}
}

int main(int argc, char** argv)
{
	knowrob::InitKnowledgeBase(argc, argv);

	auto reasonerManager = std::make_shared<knowrob::ReasonerManager>();

	// create a PrologReasoner
	knowrob::ReasonerConfiguration reasonerConfig1;
	reasonerConfig1.dataFiles.push_back(
		std::make_shared<knowrob::PrologDataFile>("tests/prolog/kb1.pl"));
	auto reasoner1 = std::make_shared<knowrob::PrologReasoner>("prolog1");
	//auto reasoner = std::make_shared<knowrob::MongologReasoner>("mongolog0");
	reasoner1->loadConfiguration(reasonerConfig1);
	reasonerManager->addReasoner(reasoner1);

	knowrob::ReasonerConfiguration reasonerConfig2;
	reasonerConfig2.dataFiles.push_back(
			std::make_shared<knowrob::PrologDataFile>("tests/prolog/kb2.pl"));
	auto reasoner2 = std::make_shared<knowrob::PrologReasoner>("prolog2");
	reasoner2->loadConfiguration(reasonerConfig2);
	reasonerManager->addReasoner(reasoner2);

	KB_INFO("running tests...");
	test_kb1_PrologReasoner(reasoner1, reasonerManager);
	KB_INFO("done with tests.");
	return 0;
}
