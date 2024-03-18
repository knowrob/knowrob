
#include <gtest/gtest.h>
#include <knowrob/Logger.h>
#include <knowrob/reasoner/mongolog/MongologReasoner.h>
#include "knowrob/reasoner/prolog/PrologTests.h"

using namespace knowrob;

namespace knowrob::testing {
	class NEEMTests : public PrologTestsBase {
	protected:

		static std::shared_ptr<knowrob::MongoKnowledgeGraph>
		createBackend(const std::string &name, const std::shared_ptr<KnowledgeBase> &kb) {
			auto kg = std::make_shared<MongoKnowledgeGraph>();
			kb->backendManager()->addBackend(name, kg);
			kg->initializeBackend(
					MongoKnowledgeGraph::DB_URI_DEFAULT,
					MongoKnowledgeGraph::DB_NAME_TESTS,
					MongoKnowledgeGraph::COLL_NAME_TRIPLES);
			return kg;
		}

		static std::shared_ptr<MongologReasoner>
		createReasoner(const std::string &name, const std::shared_ptr<KnowledgeBase> &kb,
					   const std::shared_ptr<MongoKnowledgeGraph> &db) {
			auto r = std::make_shared<MongologReasoner>();
			r->setDataBackend(db);
			kb->reasonerManager()->addReasoner(name, r);
			r->loadConfig(knowrob::PropertyTree());
			r->consult(std::filesystem::path("neems") / "__init__.pl", nullptr, false);
			r->load_rdf_xml("http://www.ease-crc.org/ont/SOMA.owl");
			return r;
		}

		// Per-test-suite set-up.
		static void SetUpTestSuite() {
			// Initialize the reasoner
			KB_INFO("MongologTests SetUpTestSuite");
			try {
				reasoner();
			} catch (std::exception &e) {
				FAIL() << "SetUpTestSuite failed: " << e.what();
			}
		}

		static void runTests(const std::string &t) {
			try {
				runPrologTests(reasoner(), t);
			} catch (std::exception &e) {
				FAIL() << "runTests failed: " << e.what();
			}
		}

		static std::shared_ptr<MongologReasoner> reasoner() {
			static std::shared_ptr<MongologReasoner> reasoner;
			static std::shared_ptr<KnowledgeBase> kb;
			static std::shared_ptr<MongoKnowledgeGraph> db;

			if (!reasoner) {
				static int reasonerIndex_ = 0;
				std::stringstream ss;
				ss << "prolog" << reasonerIndex_++;

				kb = std::make_shared<KnowledgeBase>();
				db = createBackend(ss.str(), kb);
				reasoner = createReasoner(ss.str(), kb, db);

				kb->init();
			}
			return reasoner;
		}

		static KnowledgeBase *kb() {
			return reasoner()->reasonerManager().kb();
		}

		static std::string getPath(const std::string &filename) {
			return std::filesystem::path("neems") / filename;
		}
	};
}
using namespace knowrob::testing;

TEST_F(NEEMTests, occurs) { runTests(getPath("occurs.plt")); }
TEST_F(NEEMTests, neem_logging) { runTests(getPath("NEEM.pl")); }
