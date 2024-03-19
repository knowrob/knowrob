#include <gtest/gtest.h>
#include <knowrob/knowrob.h>

int main(int argc, char **argv)
{
	knowrob::InitKnowledgeBase(argc, argv);
	testing::InitGoogleTest(&argc, argv);
	auto status = RUN_ALL_TESTS();
	knowrob::ShutdownKnowledgeBase();
	return status;
}
