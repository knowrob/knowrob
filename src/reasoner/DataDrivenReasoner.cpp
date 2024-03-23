/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/DataDrivenReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

InferredTripleContainer DataDrivenReasoner::createTriples(uint32_t count) const {
	auto triples = std::make_shared<std::vector<std::shared_ptr<FramedTriple>>>(count);
	for (uint32_t i = 0; i < count; i++) {
		(*triples)[i] = std::make_shared<FramedTripleCopy>();
		(*triples)[i]->setGraph(reasonerName()->stringForm());
	}
	return triples;
}

bool DataDrivenReasoner::InferredComparator::operator()(
		const std::shared_ptr<FramedTriple> &v0,
		const std::shared_ptr<FramedTriple> &v1) const {
	return *v0 < *v1;
}

void DataDrivenReasoner::setInferredTriples(const InferredTripleContainer &triples) {
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

void DataDrivenReasoner::addInferredTriples(const InferredTripleContainer &triples) const {
	std::vector<FramedTriplePtr> triplesVector(triples->size());
	for (size_t i = 0; i < triples->size(); i++) {
		triplesVector[i].ptr = (*triples)[i].get();
		triplesVector[i].owned = false;
	}
	reasonerManager().kb()->insertAll(triplesVector);
}

void DataDrivenReasoner::removeInferredTriples(const InferredTripleContainer &triples) const {
	std::vector<FramedTriplePtr> triplesVector(triples->size());
	for (size_t i = 0; i < triples->size(); i++) {
		triplesVector[i].ptr = (*triples)[i].get();
		triplesVector[i].owned = false;
	}
	reasonerManager().kb()->removeAll(triplesVector);
}

namespace knowrob::py {
	// this struct is needed because Reasoner has pure virtual methods
	struct DataDrivenReasonerWrap : public DataDrivenReasoner, boost::python::wrapper<DataDrivenReasoner> {
		explicit DataDrivenReasonerWrap(PyObject *p) : self(p), DataDrivenReasoner() {}

		void setDataBackend(const DataBackendPtr &backend) override {
			call_method<void>(self, "setDataBackend", backend);
		}

		bool loadConfig(const PropertyTree &config) override {
			return call_method<bool>(self, "loadConfig", config);
		}

		void start() override { call_method<void>(self, "start"); }

		void stop() override { call_method<void>(self, "stop"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<DataDrivenReasoner>() {
		using namespace boost::python;
		class_<DataDrivenReasoner, std::shared_ptr<DataDrivenReasonerWrap>, bases<Reasoner>, boost::noncopyable>
				("DataDrivenReasoner", init<>())
				.def("createTriples", &DataDrivenReasonerWrap::createTriples)
				.def("setInferredTriples", &DataDrivenReasonerWrap::setInferredTriples)
				.def("addInferredTriples", &DataDrivenReasonerWrap::addInferredTriples)
				.def("removeInferredTriples", &DataDrivenReasonerWrap::removeInferredTriples)
						// methods that must be implemented by reasoner plugins
				.def("start", &DataDrivenReasonerWrap::start)
				.def("stop", &DataDrivenReasonerWrap::stop);

		using InferredTriples = std::vector<std::shared_ptr<FramedTriple>>;
		boost::python::register_ptr_to_python<std::shared_ptr<FramedTriple>>();
		class_<InferredTriples, std::shared_ptr<InferredTriples>, boost::noncopyable>
				("InferredTriples", no_init)
				.def("__iter__", range(&InferredTriples::cbegin, &InferredTriples::cend));
	}
}
