
#include <gtest/gtest.h>
#include <knowrob/prolog/PrologReasoner.h>
#include <knowrob/logging.h>

int main(int argc, char **argv)
{
	knowrob::logging::initialize();
	testing::InitGoogleTest(&argc, argv);
	knowrob::PrologReasoner::initialize(argc, argv);
	return RUN_ALL_TESTS();
}
