
#include <iostream>

#include <spdlog/spdlog.h>
// KnowRob
#include <knowrob/reasoning/prolog/PrologReasoner.h>
#include <knowrob/reasoning/prolog/PrologQuery.h>

//using namespace knowrob;
void test_kb1_PrologReasoner(const std::shared_ptr<knowrob::PrologReasoner> &reasoner) {
	spdlog::info("?- woman(X).");
	for(auto solution : reasoner->allSolutions(
		knowrob::PrologQuery::toQuery(reasoner->readTerm("woman(X)"))))
	{
		spdlog::info("{}", solution->toString());
	}
}

int main(int argc, char** argv){
	// TODO: would be nice if this would be done "under the hood"
	//  in the constructor. but I got a segfault when I tried, not sure why.
	knowrob::PrologReasoner::initialize(argc, argv);
	
	auto reasoner = std::shared_ptr<knowrob::PrologReasoner>(
		new knowrob::PrologReasoner("test/prolog/kb1.pl"));
	reasoner->initialize();
	
	spdlog::info("running tests...");
	test_kb1_PrologReasoner(reasoner);
	spdlog::info("done with tests.");
	return 0;
}

