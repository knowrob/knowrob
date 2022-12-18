
#include <iostream>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/qa/queries.h>
#include <knowrob/qa/Blackboard.h>
#include <knowrob/reasoning/ReasonerManager.h>
#include <knowrob/reasoning/prolog/PrologReasoner.h>
#include <knowrob/reasoning/prolog/PrologQuery.h>

//using namespace knowrob;
void test_kb1_PrologReasoner(
	const std::shared_ptr<knowrob::PrologReasoner> &reasoner,
	const std::shared_ptr<knowrob::ReasonerManager> &reasonerManager)
{
	auto query1 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X)"));
	auto query3 = knowrob::PrologQuery::toQuery(reasoner->readTerm("loves(X,Y)"));
	auto query2 = knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X), loves(Y,X)"));
	
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
		std::make_shared<knowrob::PrologDataFile>("test/prolog/kb1.pl"));
	auto reasoner = std::make_shared<knowrob::PrologReasoner>();
	reasoner->initialize(reasonerConfig);
	
	// add the reasoner to the reasoner manager used by the blackboard
	auto reasonerManager = std::make_shared<knowrob::ReasonerManager>();
	reasonerManager->addReasoner(reasoner);
	
	KB_INFO("running tests...");
	test_kb1_PrologReasoner(reasoner, reasonerManager);
	KB_INFO("done with tests.");
	return 0;
}

