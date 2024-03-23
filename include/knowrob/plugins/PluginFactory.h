/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PLUGIN_FACTORY_H_
#define KNOWROB_PLUGIN_FACTORY_H_

#include <string>
#include <memory>
#include "NamedPlugin.h"

namespace knowrob {
	/**
	 * Abstract plugin factory.
	 * Provides an interface for the creation of plugin instances.
	 */
	template<class T>
	class PluginFactory {
	public:
		virtual ~PluginFactory() = default;

		/**
		 * Create a new plugin instance.
		 * @param pluginID the ID of the plugin.
		 * @return the plugin created.
		 */
		virtual std::shared_ptr<NamedPlugin<T>> create(std::string_view pluginID) = 0;

		/**
		 * @return name of the plugin type for which the factory can create instances.
		 */
		virtual std::string_view name() const = 0;
	};
}

#endif //KNOWROB_PLUGIN_FACTORY_H_
