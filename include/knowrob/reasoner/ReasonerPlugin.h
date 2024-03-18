/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_PLUGIN_H_
#define KNOWROB_REASONER_PLUGIN_H_

#include <string>
#include <memory>
#include "knowrob/reasoner/ReasonerFactory.h"

namespace knowrob {
	/**
	 * A reasoner factory that uses a shared library for creation of reasoner instances.
	 * Reasoner plugins are usually defined in shared libraries through the REASONER_PLUGIN macro.
	 * They need to expose a set of functions that are used as an entry point for loading the plugin.
	 */
	class ReasonerPlugin : public ReasonerFactory {
	public:
		/**
		 * @param dllPath the name or path of the shared library.
		 */
		explicit ReasonerPlugin(std::string_view dllPath);

		~ReasonerPlugin() override;

		/**
		 * Cannot be copy-assigned.
		 */
		ReasonerPlugin(const ReasonerPlugin &) = delete;

		/**
		 * @return true if the shared library was loaded successfully.
		 */
		bool isLoaded();

		/**
		 * Try loading the shared library from filesystem.
		 * Note that, on unix-based systems, the LD_LIBRARY_PATH environment variable is used to locate the library.
		 * @return true on success.
		 */
		bool loadDLL();

		// Override ReasonerFactory
		std::shared_ptr<Reasoner> createReasoner(std::string_view reasonerID) override;

		// Override ReasonerFactory
		std::string_view name() const override { return name_; };

	protected:
		const std::string dllPath_;
		std::string name_;
		// handle of opened library
		void *handle_;

		// a factory function used to create new instances of a reasoner.
		std::shared_ptr<Reasoner> (*create_)(std::string_view reasonerID);

		// a function that returns the name of the plugin
		char *(*get_name_)();
	};
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
        std::shared_ptr<knowrob::IReasoner> knowrob_createReasoner(std::string_view reasonerID) \
            { return std::make_shared<classType>(reasonerID); } \
        const char* knowrob_getPluginName() { return pluginName; } }

#endif //KNOWROB_REASONER_PLUGIN_H_
