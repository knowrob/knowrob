
#include <gtest/gtest.h>
#include <knowrob/logging.h>
#include <knowrob/mongolog/MongologReasoner.h>

using namespace knowrob;

class NEEMTests: public PrologTests<knowrob::MongologReasoner> {
protected:
	static void SetUpTestSuite() {
		reasoner()->consult(std::filesystem::path("neems") / "__init__.pl", nullptr, false);
		reasoner()->load_rdf_xml("http://www.ease-crc.org/ont/SOMA.owl");
	}
	static std::string getPath(const std::string &filename) {
		return std::filesystem::path("neems") / filename;
	}
};

TEST_F(NEEMTests, occurs) { runTests(getPath("occurs.plt")); }
TEST_F(NEEMTests, neem_logging) { runTests(getPath("NEEM.pl")); }
