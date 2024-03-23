/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_DRIVEN_REASONER_H
#define KNOWROB_DATA_DRIVEN_REASONER_H

#include "Reasoner.h"
#include "ReasonerEvent.h"

namespace knowrob {
	/**
	 * A reasoner that is driven by data, i.e., it starts with the data and infers
	 * additional knowledge e.g. by applying rules to the data.
	 * This is in contrast to a goal-driven reasoner, which is driven by queries.
	 * The data is taken from a data backend, which is associated with the reasoner
	 * before the reasoner is started.
	 */
	class DataDrivenReasoner : public Reasoner {
	public:
		/**
		 * Features of the reasoner.
		 */
		enum Feature {
			// The reasoner does not have any special features.
			NothingSpecial = 1 << 0,
			// The reasoner updates itself, thus `update` does not need to be called externally.
			UpdatesItself = 1 << 1,
			// The reasoner invalidates itself, thus `update` does not need to be called periodically
			// in case no invalidation happens.
			InvalidatesItself = 1 << 2
		};

		DataDrivenReasoner();

		/**
		 * Enable a feature of the reasoner.
		 * @param feature a feature.
		 */
		void enableFeature(Feature feature);

		/**
		 * @return true if the reasoner has a feature.
		 */
		bool hasFeature(Feature feature) const;

		/**
		 * Set the update rate of the reasoner.
		 * @param intervalInSeconds the update interval in seconds.
		 */
		void setUpdateInterval(double intervalInSeconds);

		/**
		 * @return the update interval of the reasoner.
		 */
		auto updateInterval() const { return updateInterval_; }

		/**
		 * Trigger an event on the reasoner.
		 * This is the main way how the reasoner informs the rest of the system
		 * about its result and state.
		 * @param event a reasoner event.
		 */
		void emit(const std::shared_ptr<reasoner::Event> &event);

		/**
		 * Update the reasoner.
		 * This function is called periodically to update the reasoner's state
		 * in case the reasoner does not update itself (see `enableFeature`).
		 */
		virtual void update() = 0;

		/**
		 * Start calling the update function periodically.
		 */
		virtual void start();

		/**
		 * Stop calling the update function.
		 */
		virtual void stop();

	protected:
		bool isUpdateQueued_ = false;
		bool isRunning_ = false;
		bool isInvalidated_ = true;
		std::chrono::duration<double> updateInterval_ = std::chrono::seconds(1);
		uint32_t features_ = NothingSpecial;
		std::set<FramedTriplePtr> inferredTriples_;
		std::shared_ptr<ThreadPool::Runner> updateRunner_;
		std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate_;

		void processAssertion(const std::vector<FramedTriplePtr> &triples);

		void processRetraction(const std::vector<FramedTriplePtr> &triples);

		void processReplacement(const std::vector<FramedTriplePtr> &triples);

		void processInvalidation();

		void setReasonerOrigin(const std::vector<FramedTriplePtr> &triples);

		void doUpdate();

		void queueUpdate();
	};

	using DataDrivenReasonerPtr = std::shared_ptr<DataDrivenReasoner>;
}

#endif //KNOWROB_DATA_DRIVEN_REASONER_H
