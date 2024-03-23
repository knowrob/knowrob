/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/DataDrivenReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

DataDrivenReasoner::DataDrivenReasoner() {
	using StopChecker = ThreadPool::LambdaRunner::StopChecker;
	using Runner = ThreadPool::LambdaRunner;
	updateRunner_ = std::make_shared<Runner>([this](const StopChecker &) { doUpdate(); });
}

void DataDrivenReasoner::enableFeature(Feature feature) {
	features_ = features_ | static_cast<uint32_t>(feature);
}

bool DataDrivenReasoner::hasFeature(Feature feature) const {
	return (features_ & static_cast<uint32_t>(feature)) == static_cast<uint32_t>(feature);
}

void DataDrivenReasoner::setUpdateInterval(double intervalInSeconds) {
	updateInterval_ = std::chrono::duration<double>(intervalInSeconds);
}

void DataDrivenReasoner::queueUpdate() {
	if (isUpdateQueued_) return;
	isUpdateQueued_ = true;
	DefaultThreadPool()->pushWork(
			updateRunner_,
			[](const std::exception &exc) {
				KB_ERROR("Error in reasoner update: {}", exc.what());
			});
}

void DataDrivenReasoner::doUpdate() {
	// do the update
	update();
	auto now = std::chrono::high_resolution_clock::now();
	isUpdateQueued_ = false;
	isInvalidated_ = false;

	if (!hasFeature(UpdatesItself) && !hasFeature(InvalidatesItself)) {
		// if the reasoner does not update itself and does not invalidate itself, then
		// periodically update the reasoner.
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate_);
		if (duration > updateInterval_) {
			// we cannot keep up with the update rate, so directly re-queue the update.
			queueUpdate();
		} else {
			auto remaining = std::chrono::milliseconds(
					static_cast<int>(updateInterval_.count()) - duration.count());
			// Create a timer that triggers the update after the remaining time.
			// It is not optimal to create a new thread for each update though,
			// but it would be good to stick to the worker thread pool for performing the
			// update. However, the worker thread pool does not support delayed execution yet.
			// Note: this is done so to avoid calling sleep for the worker thread
			//       as it would block it from being used for other tasks.
			std::thread timeout([&remaining, this]() {
				std::this_thread::sleep_for(remaining);
				queueUpdate();
			});
			timeout.detach();
		}
	}

	lastUpdate_ = now;
}

void DataDrivenReasoner::start() {
	// Only start if not already running
	if (isRunning_) return;
	isRunning_ = true;
	// update the reasoner if it does not update itself
	if (!hasFeature(UpdatesItself)) {
		queueUpdate();
	}
}

void DataDrivenReasoner::stop() {
	if (!isRunning_) return;
	isRunning_ = false;
	// stop the periodic updating
	if (!hasFeature(UpdatesItself)) {
		if (updateRunner_ && !updateRunner_->isTerminated()) {
			updateRunner_->stop(false);
		}
	}
}

void DataDrivenReasoner::emit(const std::shared_ptr<reasoner::Event> &event) {
	switch (event->eventType()) {
		case reasoner::Event::Assertion:
			processAssertion(std::static_pointer_cast<reasoner::AssertionEvent>(event)->triples());
			break;
		case reasoner::Event::Retraction:
			processRetraction(std::static_pointer_cast<reasoner::RetractionEvent>(event)->triples());
			break;
		case reasoner::Event::Replacement:
			processReplacement(std::static_pointer_cast<reasoner::ReplacementEvent>(event)->triples());
			break;
		case reasoner::Event::Invalidation:
			processInvalidation();
			break;
	}
}

void DataDrivenReasoner::processInvalidation() {
	if (!hasFeature(InvalidatesItself)) {
		KB_WARN("Reasoner has no feature to invalidate itself, but still generated an invalidation event. Ignoring.");
		return;
	}
	if (!isInvalidated_) {
		isInvalidated_ = true;
		queueUpdate();
	}
}

void DataDrivenReasoner::setReasonerOrigin(const std::vector<FramedTriplePtr> &triples) {
	for (auto &triple: triples) {
		triple.ptr->setGraph(reasonerName()->stringForm());
	}
}

void DataDrivenReasoner::processAssertion(const std::vector<FramedTriplePtr> &triples) {
	setReasonerOrigin(triples);
	reasonerManager().kb()->insertAll(triples);
}

void DataDrivenReasoner::processRetraction(const std::vector<FramedTriplePtr> &triples) {
	setReasonerOrigin(triples);
	reasonerManager().kb()->removeAll(triples);
}

void DataDrivenReasoner::processReplacement(const std::vector<FramedTriplePtr> &triples) {
	setReasonerOrigin(triples);

	if (inferredTriples_.empty()) {
		inferredTriples_.insert(triples.begin(), triples.end());
		reasonerManager().kb()->insertAll(triples);
	} else {
		auto &oldTriples = inferredTriples_;
		// ensure that input triples are sorted which is required for set_difference
		std::set<FramedTriplePtr> newTriples(triples.begin(), triples.end());

		std::vector<FramedTriplePtr> triplesToRemove, triplesToAdd;
		// old inferences without new inferences are the ones that do not hold anymore.
		std::set_difference(oldTriples.begin(), oldTriples.end(),
							newTriples.begin(), newTriples.end(),
							std::inserter(triplesToRemove, triplesToRemove.begin()));
		// new inferences without old inferences are the ones that are really new.
		std::set_difference(newTriples.begin(), newTriples.end(),
							oldTriples.begin(), oldTriples.end(),
							std::inserter(triplesToAdd, triplesToAdd.begin()));
		// update the set of inferred triples.
		for (auto &triple: triplesToRemove) {
			inferredTriples_.erase(triple);
		}
		inferredTriples_.insert(triplesToAdd.begin(), triplesToAdd.end());
		// update the knowledge base
		if (!triplesToAdd.empty()) {
			reasonerManager().kb()->insertAll(triplesToAdd);
		}
		if (!triplesToRemove.empty()) {
			reasonerManager().kb()->removeAll(triplesToRemove);
		}
	}
}

namespace knowrob::py {
	// this struct is needed because Reasoner has pure virtual methods
	struct DataDrivenReasonerWrap : public DataDrivenReasoner, boost::python::wrapper<DataDrivenReasoner> {
		explicit DataDrivenReasonerWrap(PyObject *p) : self(p), DataDrivenReasoner() {}

		void setDataBackend(const DataBackendPtr &backend) override {
			call_method<void>(self, "setDataBackend", backend);
		}

		bool initializeReasoner(const PropertyTree &config) override {
			return call_method<bool>(self, "initializeReasoner", config);
		}

		void start() override {
			start_default();
			// Note: in case there is no overwrite, this will call the default implementation
			//       a second time. But it is guarded by a check in the default implementation, so no problem.
			call_method<void>(self, "start");
		}

		void start_default() { return this->DataDrivenReasoner::start(); }

		void stop() override {
			call_method<void>(self, "stop");
			// Note: in case there is no overwrite, this will call the default implementation
			//       a second time. But it is guarded by a check in the default implementation, so no problem.
			stop_default();
		}

		void stop_default() { return this->DataDrivenReasoner::stop(); }

		void update() override { call_method<void>(self, "update"); }

	private:
		PyObject *self;
	};

	template<>
	void createType<DataDrivenReasoner>() {
		using namespace boost::python;
		enum_<DataDrivenReasoner::Feature>("DataDrivenReasonerFeature")
				.value("NothingSpecial", DataDrivenReasoner::NothingSpecial)
				.value("UpdatesItself", DataDrivenReasoner::UpdatesItself)
				.value("InvalidatesItself", DataDrivenReasoner::InvalidatesItself)
				.export_values();

		class_<DataDrivenReasoner, std::shared_ptr<DataDrivenReasonerWrap>, bases<Reasoner>, boost::noncopyable>
				("DataDrivenReasoner", init<>())
				.def("enableFeature", &DataDrivenReasonerWrap::enableFeature)
				.def("hasFeature", &DataDrivenReasonerWrap::hasFeature)
				.def("emit", &DataDrivenReasonerWrap::emit)
				.def("setUpdateInterval", &DataDrivenReasonerWrap::setUpdateInterval)
				.def("updateInterval", &DataDrivenReasonerWrap::updateInterval)
						// methods that must be implemented by reasoner plugins
				.def("update", &DataDrivenReasonerWrap::update)
				.def("start", &DataDrivenReasonerWrap::start, &DataDrivenReasonerWrap::start_default)
				.def("stop", &DataDrivenReasonerWrap::stop, &DataDrivenReasonerWrap::stop_default);
	}
}
