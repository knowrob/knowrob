/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_H_
#define KNOWROB_REASONER_H_

#include <memory>
#include "knowrob/PropertyTree.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/sources//DataSource.h"
#include "knowrob/sources/DataSourceHandler.h"
#include "knowrob/backend/Backend.h"
#include "knowrob/plugins/NamedPlugin.h"

namespace knowrob {
	// forward declaration
	class ReasonerManager;

	/**
	 * A reasoner is a component that can infer new knowledge.
	 * The inference process may refer to extensional data which is stored in a DataBackend.
	 * Note that a reasoner is also a data source handler, i.e. data which is needed
	 * by the reasoner to operate which is not stored in a backend.
	 */
	class Reasoner : public DataSourceHandler {
	public:
		Reasoner() : reasonerManager_(nullptr) {}

		virtual ~Reasoner() = default;

		/**
		 * @return a term representing the reasoner name.
		 */
		auto &reasonerName() const { return t_reasonerName_; }

		/**
		 * @return the reasoner manager associated with this reasoner.
		 */
		ReasonerManager &reasonerManager() const;

		/**
		 * Evaluate a lambda function in a worker thread.
		 * @param fn a function to be executed.
		 */
		void pushWork(const std::function<void(void)> &fn);

		/**
		 * Set the data backend of this reasoner.
		 */
		virtual void setDataBackend(const DataBackendPtr &backend) = 0;

		/**
		 * Load a reasoner configuration.
		 * The knowledge base system only calls this function once for each reasoner instance.
		 * @param config a ReasonerConfig object.
		 */
		virtual bool loadConfig(const PropertyTree &config) = 0;

	private:
		AtomPtr t_reasonerName_;
		ReasonerManager *reasonerManager_;

		void setReasonerManager(ReasonerManager *reasonerManager) { reasonerManager_ = reasonerManager; }

		void setReasonerName(std::string_view name) { t_reasonerName_ = Atom::Tabled(name); }

		friend class ReasonerManager;
	};

	using NamedReasoner = NamedPlugin<Reasoner>;
	using ReasonerFactory = PluginFactory<Reasoner>;
	using ReasonerPtr = std::shared_ptr<Reasoner>;
}

/**
 * Define a reasoner plugin.
 * The macro generates two functions that are used as entry points for loading the plugin.
 * First, a factory function is defined that creates instances of @classType.
 * This will only work when @classType has a single argument constructor that
 * accepts a string as argument (the reasoner instance ID).
 * Second, a function is generated that exposes the plugin name.
 * @param classType the type of the reasoner, must be a subclass of IReasoner
 * @param pluginName a plugin identifier, e.g. the name of the reasoner type.
 */
#define REASONER_PLUGIN(classType, pluginName) extern "C" { \
        std::shared_ptr<knowrob::Reasoner> knowrob_createPlugin(std::string_view pluginID) \
            { return std::make_shared<classType>(pluginID); } \
        const char* knowrob_getPluginName() { return pluginName; } }

#endif //KNOWROB_REASONER_H_
