
#include <iostream>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/queries.h>
#include <knowrob/Blackboard.h>
#include <knowrob/ReasonerManager.h>
#include <knowrob/prolog/PrologQuery.h>
#include <knowrob/prolog/PrologReasoner.h>
#include <knowrob/mongolog/MongologReasoner.h>

//using namespace knowrob;
void test_kb1_PrologReasoner(
	const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
	const std::shared_ptr<knowrob::ReasonerManager> &reasonerManager)
{
	auto query1 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X)"));
	auto query2 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X), loves(Y,X)"));
	auto query3 = knowrob::PrologQuery::toQuery(reasoner->readTerm("triple(X,Y,Z)"));

	KB_INFO("PrologReasoner query: {}", (*query1.get()));
	for(auto solution : reasoner->allSolutions(query1))
	{
		KB_INFO("{}", (*solution.get()));
	}

	KB_INFO("PrologReasoner query: {}", (*query2.get()));
	for(auto solution : reasoner->allSolutions(query2))
	{
		KB_INFO("{}", (*solution.get()));
	}

	KB_INFO("PrologReasoner query: {}", (*query3.get()));
	for(auto solution : reasoner->allSolutions(query3))
	{
		KB_INFO("{}", (*solution.get()));
	}

	KB_INFO("Blackboard query: {}", (*query1.get())); {
		auto bbq = std::make_shared<knowrob::QueryResultQueue>();
		auto bb = std::make_shared<knowrob::Blackboard>(reasonerManager, bbq, query1);
		bb->start();
		while(true) {
			auto solution = bbq->pop_front();
			if(knowrob::QueryResultStream::isEOS(solution)) break;
			KB_INFO("{}", (*solution.get()));
		}
	}

	KB_INFO("Blackboard query: {}", (*query2.get())); {
		auto bbq = std::make_shared<knowrob::QueryResultQueue>();
		auto bb = std::make_shared<knowrob::Blackboard>(reasonerManager, bbq, query2);
		bb->start();
		while(true) {
			auto solution = bbq->pop_front();
			if(knowrob::QueryResultStream::isEOS(solution)) break;
			KB_INFO("{}", (*solution.get()));
		}
	}
}


int main(int argc, char** argv)
{
	knowrob::logging::initialize();
	// TODO: would be nice if this would be done "under the hood"
	//  in the constructor. but I got a segfault when I tried, not sure why.
	knowrob::PrologReasoner::initialize(argc, argv);
	
	// create a PrologReasoner
	knowrob::ReasonerConfiguration reasonerConfig;
	reasonerConfig.dataFiles.push_back(
		std::make_shared<knowrob::PrologDataFile>("tests/prolog/kb1.pl"));
	//auto reasoner = std::make_shared<knowrob::PrologReasoner>();
	auto reasoner = std::make_shared<knowrob::MongologReasoner>();
	reasoner->initialize(reasonerConfig);
	
	// add the reasoner to the reasoner manager used by the blackboard
	auto reasonerManager = std::make_shared<knowrob::ReasonerManager>();
	reasonerManager->addReasoner(reasoner);
	
	KB_INFO("running tests...");
	test_kb1_PrologReasoner(reasoner, reasonerManager);
	KB_INFO("done with tests.");
	return 0;
}
