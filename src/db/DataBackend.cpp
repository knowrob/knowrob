#include "knowrob/db/DataBackend.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

namespace knowrob::py {
	struct DataBackendWrap : public DataBackend, boost::python::wrapper<DataBackend> {
		explicit DataBackendWrap(PyObject *p) : self(p), DataBackend() {}

		bool initializeBackend(const ReasonerConfig &config) override {
			return call_method<bool>(self, "initializeBackend", config);
		}

		bool insertOne(const FramedTriple &triple) override {
			return call_method<bool>(self, "insertOne", &triple);
		}

		bool insertAll(const semweb::TripleContainerPtr &triples) override {
			return call_method<bool>(self, "insertAll", triples);
		}

		bool removeOne(const FramedTriple &triple) override {
			return call_method<bool>(self, "removeOne", &triple);
		}

		bool removeAll(const semweb::TripleContainerPtr &triples) override {
			return call_method<bool>(self, "removeAll", triples);
		}

		bool removeAllWithOrigin(std::string_view origin) override {
			return call_method<bool>(self, "removeAllWithOrigin", origin.data());
		}

		bool removeAllMatching(const FramedTriplePatternPtr &query) override {
			return call_method<int>(self, "removeAllMatching", query);
		}

	private:
		PyObject *self;
	};

	template<>
	void createType<DataBackend>() {
		using namespace boost::python;
		class_<DataBackend, std::shared_ptr<DataBackendWrap>, bases<DataSourceHandler>, boost::noncopyable>
				("DataBackend", init<>())
				// methods that must be implemented by backend plugins
				.def("initializeBackend", pure_virtual(&DataBackendWrap::initializeBackend))
				.def("insertOne", pure_virtual(&DataBackendWrap::insertOne))
				.def("insertAll", pure_virtual(&DataBackendWrap::insertAll))
				.def("removeAll", pure_virtual(&DataBackendWrap::removeAll))
				.def("removeOne", pure_virtual(&DataBackendWrap::removeOne))
				.def("removeAllWithOrigin", pure_virtual(&DataBackendWrap::removeAllWithOrigin))
				.def("removeAllMatching", pure_virtual(&DataBackendWrap::removeAllMatching));
	}
}
