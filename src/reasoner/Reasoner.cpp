/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/Reasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/ReasonerError.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

Reasoner::Reasoner()
		: reasonerManager_(nullptr) {
}

void Reasoner::setReasonerManager(ReasonerManager *reasonerManager) {
	reasonerManager_ = reasonerManager;
}

void Reasoner::setReasonerName(std::string_view name) {
	t_reasonerName_ = Atom::Tabled(name);
}

KnowledgeBase *Reasoner::kb() const {
	return reasonerManager().kb();
}

ReasonerManager &Reasoner::reasonerManager() const {
	if (reasonerManager_) {
		return *reasonerManager_;
	} else {
		throw ReasonerError("ReasonerManager not found.");
	}
}

std::shared_ptr<Vocabulary> Reasoner::vocabulary() const {
	return reasonerManager().kb()->vocabulary();
}

std::shared_ptr<ImportHierarchy> Reasoner::importHierarchy() const {
	return reasonerManager().kb()->importHierarchy();
}

namespace knowrob {
	class ReasonerTask : public ThreadPool::Runner {
	public:
		explicit ReasonerTask(const std::function<void()> &fn) : fn_(fn) {}

		void run() override { fn_(); }

	protected:
		std::function<void()> fn_;
	};
}

void Reasoner::pushWork(const std::function<void(void)> &fn) {
	// TODO: add support for stop request flag. For this the lambda needs to take an additional parameter.
	//       which itself is a function that returns the stop request flag of the worker.
	auto runner = std::make_shared<ReasonerTask>(fn);
	DefaultThreadPool()->pushWork(runner, [](const std::exception &e) {
		KB_ERROR("Error in reasoner worker thread: {}", e.what());
	});
}

InferredTripleContainer Reasoner::createTriples(uint32_t count) const {
	auto triples = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>(count);
	for (uint32_t i = 0; i < count; i++) {
		(*triples)[i] = std::make_shared<FramedTripleCopy>();
		(*triples)[i]->setGraph(reasonerName());
	}
	return triples;
}

bool Reasoner::InferredComparator::operator()(
		const std::shared_ptr<FramedTriple> &v0,
		const std::shared_ptr<FramedTriple> &v1) const {
	return *v0 < *v1;
}

void Reasoner::setInferredTriples(const InferredTripleContainer &triples) {
	if (inferredTriples_.empty()) {
		inferredTriples_.insert(triples->begin(), triples->end());
		addInferredTriples(triples);
	} else {
		auto &oldTriples = inferredTriples_;
		// ensure that input triples are sorted which is required for set_difference
		InferredeSet newTriples(triples->begin(), triples->end());

		auto triplesToRemove = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>();
		auto triplesToAdd = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>();
		// old inferences without new inferences are the ones that do not hold anymore.
		std::set_difference(oldTriples.begin(), oldTriples.end(),
							newTriples.begin(), newTriples.end(),
							std::inserter(*triplesToRemove, triplesToRemove->begin()));
		// new inferences without old inferences are the ones that are really new.
		std::set_difference(newTriples.begin(), newTriples.end(),
							oldTriples.begin(), oldTriples.end(),
							std::inserter(*triplesToAdd, triplesToAdd->begin()));
		// update the set of inferred triples.
		for (auto &triple: *triplesToRemove) {
			inferredTriples_.erase(triple);
		}
		inferredTriples_.insert(triplesToAdd->begin(), triplesToAdd->end());
		// update the knowledge base
		if (!triplesToAdd->empty()) {
			addInferredTriples(triplesToAdd);
		}
		if (!triplesToRemove->empty()) {
			removeInferredTriples(triplesToRemove);
		}
	}
}

void Reasoner::addInferredTriples(const InferredTripleContainer &triples) const {
	std::vector<FramedTriplePtr> triplesVector(triples->size());
	for (size_t i = 0; i < triples->size(); i++) {
		triplesVector[i].ptr = (*triples)[i].get();
		triplesVector[i].owned = false;
	}
	kb()->insertAll(triplesVector);
}

void Reasoner::removeInferredTriples(const InferredTripleContainer &triples) const {
	std::vector<FramedTriplePtr> triplesVector(triples->size());
	for (size_t i = 0; i < triples->size(); i++) {
		triplesVector[i].ptr = (*triples)[i].get();
		triplesVector[i].owned = false;
	}
	kb()->removeAll(triplesVector);
}

PredicateDescriptionPtr Reasoner::getLiteralDescription(const FramedTriplePattern &literal) {
	if (literal.propertyTerm() && literal.propertyTerm()->termType() == TermType::ATOMIC) {
		auto p = std::static_pointer_cast<Atomic>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->stringForm().data(), 2));
	} else {
		return {};
	}
}

namespace knowrob::py {
	// this struct is needed because Reasoner has pure virtual methods
	struct ReasonerWrap : public Reasoner, boost::python::wrapper<Reasoner> {
		explicit ReasonerWrap(PyObject *p) : self(p), Reasoner() {}

		void setDataBackend(const DataBackendPtr &backend) override {
			call_method<void>(self, "setDataBackend", backend);
		}

		bool loadConfig(const ReasonerConfig &config) override {
			return call_method<bool>(self, "loadConfig", config);
		}

		PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) override {
			return call_method<PredicateDescriptionPtr>(self, "getDescription", indicator);
		}

		TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override {
			return call_method<TokenBufferPtr>(self, "submitQuery", literal, ctx);
		}

		void start() override { call_method<void>(self, "start"); }

		void stop() override { call_method<void>(self, "stop"); }

	private:
		PyObject *self;
	};

	struct ReasonerWithBackendWrap :
			public ReasonerWithBackend,
			boost::python::wrapper<ReasonerWithBackend> {
		explicit ReasonerWithBackendWrap(PyObject *p)
				: self(p), ReasonerWithBackend() {}

		// TODO: below duplicates code of ReasonerWrap and DataBackendWrap.
		//    I could not find a way to make it work with multiple inheritance.
		//    It seems boost::python::extract<T> can only retrieve one type of an object,
		//    so a workaround is needed.

		bool loadConfig(const ReasonerConfig &config) override {
			return knowrob::py::call_method<bool>(self, "loadConfig", config);
		}

		bool initializeBackend(const ReasonerConfig &config) override {
			return knowrob::py::call_method<bool>(self, "initializeBackend", config);
		}

		bool insertOne(const FramedTriple &triple) override {
			return knowrob::py::call_method<bool>(self, "insertOne", &triple);
		}

		bool insertAll(const TripleContainerPtr &triples) override {
			return knowrob::py::call_method<bool>(self, "insertAll", triples);
		}

		bool removeOne(const FramedTriple &triple) override {
			return knowrob::py::call_method<bool>(self, "removeOne", &triple);
		}

		bool removeAll(const TripleContainerPtr &triples) override {
			return knowrob::py::call_method<bool>(self, "removeAll", triples);
		}

		bool removeAllWithOrigin(std::string_view origin) override {
			return knowrob::py::call_method<bool>(self, "removeAllWithOrigin", origin.data());
		}

		PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) override {
			return knowrob::py::call_method<PredicateDescriptionPtr>(self, "getDescription", indicator);
		}

		TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) override {
			return knowrob::py::call_method<TokenBufferPtr>(self, "submitQuery", literal, ctx);
		}

		void start() override { knowrob::py::call_method<void>(self, "start"); }

		void stop() override { knowrob::py::call_method<void>(self, "stop"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<Reasoner>() {
		using namespace boost::python;
		class_<ReasonerConfig, std::shared_ptr<ReasonerConfig>>("ReasonerConfiguration", init<>())
				.def("__iter__", range(&ReasonerConfig::begin, &ReasonerConfig::end))
				.def("get", &ReasonerConfig::get)
				.def("dataSources", &ReasonerConfig::dataSources, return_value_policy<copy_const_reference>());
		class_<Reasoner, std::shared_ptr<ReasonerWrap>, bases<DataSourceHandler>, boost::noncopyable>
				("Reasoner", init<>())
				.def("createTriples", &ReasonerWrap::createTriples)
				.def("pushWork", +[](Reasoner &x, object &fn) { x.pushWork(fn); })
				.def("setInferredTriples", &ReasonerWrap::setInferredTriples)
				.def("addInferredTriples", &ReasonerWrap::addInferredTriples)
				.def("removeInferredTriples", &ReasonerWrap::removeInferredTriples)
						// methods that must be implemented by reasoner plugins
				.def("loadConfig", &ReasonerWrap::loadConfig)
				.def("setDataBackend", &ReasonerWrap::setDataBackend)
				.def("getDescription", &ReasonerWrap::getDescription)
				.def("start", &ReasonerWrap::start)
				.def("stop", &ReasonerWrap::stop)
				.def("submitQuery", &ReasonerWrap::submitQuery);
		class_<ReasonerWithBackend, std::shared_ptr<ReasonerWithBackendWrap>, bases<Reasoner, DataBackend>, boost::noncopyable>
				("ReasonerWithBackend", init<>());

		using InferredTriples = std::vector<std::shared_ptr<FramedTriple>>;
		boost::python::register_ptr_to_python<std::shared_ptr<FramedTriple>>();
		class_<InferredTriples, std::shared_ptr<InferredTriples>, boost::noncopyable>
				("InferredTriples", no_init)
				.def("__iter__", range(&InferredTriples::cbegin, &InferredTriples::cend));
	}
}
