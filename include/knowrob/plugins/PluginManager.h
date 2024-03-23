/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PLUGIN_MANAGER_H_
#define KNOWROB_PLUGIN_MANAGER_H_

#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include "knowrob/plugins/TypedPluginFactory.h"
#include "knowrob/plugins/PluginLibrary.h"
#include "knowrob/plugins/PluginModule.h"
#include "knowrob/Logger.h"

namespace knowrob {
	/**
	 * Manages a set of available plugins.
	 * @tparam T the type of the plugin.
	 */
	template<class T>
	class PluginManager {
	public:
		PluginManager() {
			std::lock_guard<std::mutex> scoped_lock(staticMutex_);
			pluginManagers_ = pluginManagers();
			managerID_ = (managerIDCounter()++);
			(*pluginManagers_)[managerID_] = this;
		}

		~PluginManager() {
			std::lock_guard<std::mutex> scoped_lock(staticMutex_);
			pluginManagers_->erase(managerID_);
		}

		/**
		 * @return the ID of this manager.
		 */
		auto managerID() const { return managerID_; }

		/**
		 * @return map of all plugins defined by this manager.
		 */
		const auto &plugins() const { return pluginPool_; }

		/**
		 * @param managerID the ID of a plugin manager
		 * @return the plugin manager, or nullptr if ID is unknown
		 */
		static PluginManager<T> *getManager(uint32_t managerID) {
			auto it = pluginManagers()->find(managerID);
			if (it != pluginManagers()->end()) {
				return it->second;
			} else {
				return nullptr;
			}
		}

		/**
		 * @param pluginID a backend ID string.
		 * @return a plugin instance or a null pointer reference.
		 */
		std::shared_ptr<NamedPlugin<T>> getPluginWithID(std::string_view pluginID) {
			auto it = pluginPool_.find(pluginID);
			if (it != pluginPool_.end()) {
				return it->second;
			} else {
				return {};
			}
		}

		/**
		 * Add a plugin factory to the manager.
		 * Note that factories for shared libraries are created on the fly, and thus
		 * do not need to be added manually.
		 * @param typeName the name of the plugin type
		 * @param factory a plugin factory
		 */
		static bool addFactory(std::string_view typeName, const std::shared_ptr<PluginFactory<T>> &factory) {
			auto &factories = pluginFactories();
			if (factories.find(typeName) != factories.end()) {
				KB_WARN("overwriting factory for plugin type '{}'", typeName);
			}
			factories.emplace(typeName, factory);
			return true;
		}

		/**
		 * Add a typed plugin factory to the manager.
		 * @param typeName the name of the plugin type
		 * @param factory a plugin factory
		 */
		template<class U>
		static bool addFactory(std::string_view typeName) {
			return addFactory(typeName, std::make_shared<TypedPluginFactory<U, T>>(typeName));
		}

		/**
		 * Load a new plugin instance into the plugin manager.
		 * The type of the plugin is determined based on either the value of
		 * "type", "lib" or "module" in the property tree root. The tree is further used
		 * to generate a plugin configuration.
		 * Plugin factories for libraries are created on the fly, the ones
		 * for built-in plugin types need to be added to the plugin manager before.
		 * @param config a property tree holding a plugin configuration
		 */
		virtual std::shared_ptr<NamedPlugin<T>> loadPlugin(const boost::property_tree::ptree &config) = 0;

		/**
		 * Add a plugin to this manager.
		 * @reasoner a plugin.
		 */
		virtual std::shared_ptr<NamedPlugin<T>>
		addPlugin(std::string_view reasonerID, const std::shared_ptr<T> &reasoner) = 0;

	protected:
		std::shared_ptr<std::map<uint32_t, PluginManager *>> pluginManagers_;
		// mutex used to interact with static variables
		std::mutex staticMutex_;
		// an identifier for this manager
		uint32_t managerID_;
		// a counter used to generate unique IDs
		uint32_t pluginIndex_ = 0;

		// pool of all plugin instances created via this manager
		// maps reasoner ID to reasoner instance.
		std::map<std::string_view, std::shared_ptr<NamedPlugin<T>>, std::less<>> pluginPool_;
		// maps plugin names to factories used to create plugin instances
		std::map<std::string, std::shared_ptr<PluginLibrary<T>>, std::less<>> loadedPlugins_;
		std::map<std::string, std::shared_ptr<PluginModule<T>>, std::less<>> loadedModules_;

		// counts number of initialized managers
		static uint32_t &managerIDCounter() {
			static uint32_t val = 0;
			return val;
		}

		// a reference to the map of plugin factories
		static auto &pluginFactories() {
			static std::map<std::string, std::shared_ptr<PluginFactory<T>>, std::less<>> factories;
			return factories;
		}

		// maps manager id to manager
		static auto &pluginManagers() {
			static auto val = std::make_shared<std::map<uint32_t, PluginManager *>>();
			return val;
		}

		std::shared_ptr<PluginLibrary<T>> loadSharedLibrary(std::string_view path) {
			auto absPath = std::filesystem::absolute(path);
			auto it = loadedPlugins_.find(absPath);
			if (it == loadedPlugins_.end()) {
				auto p = std::make_shared<PluginLibrary<T>>(absPath.c_str());
				auto jt = loadedPlugins_.insert(std::pair<std::string,
						std::shared_ptr<PluginLibrary<T>>>(absPath, p));
				if (jt.first->second->loadDLL()) {
					return jt.first->second;
				}
			} else if (it->second->isLoaded()) {
				return it->second;
			}
			KB_WARN("Failed to open plugin library at path '{}'.", path);
			return {};
		}

		std::shared_ptr<PluginModule<T>> loadPythonModule(std::string_view path, std::string_view type) {
			auto it = loadedModules_.find(path);
			if (it == loadedModules_.end()) {
				auto p = std::make_shared<PluginModule<T>>(path, type);
				auto jt = loadedModules_.insert(std::pair<std::string,
						std::shared_ptr<PluginModule<T>>>(path, p));
				if (jt.first->second->loadModule()) {
					return jt.first->second;
				}
			} else if (it->second->isLoaded()) {
				return it->second;
			}
			KB_WARN("Failed to open plugin module at path '{}'.", path);
			return {};
		}

		std::shared_ptr<PluginFactory<T>> findFactory(const boost::property_tree::ptree &config) {
			auto lib = config.get_optional<std::string>("lib");
			auto module = config.get_optional<std::string>("module");
			auto type = config.get_optional<std::string>("type");

			if (lib.has_value()) {
				return loadSharedLibrary(lib.value());
			} else if (module.has_value()) {
				if (type.has_value()) {
					return loadPythonModule(module.value(), type.value());
				} else {
					KB_WARN("modules require type key in settings, but it's missing for module '{}'.", module.value());
				}
			} else if (type.has_value()) {
				// map type name to a factory
				const auto &it = pluginFactories().find(type.value());
				if (it == pluginFactories().end()) {
					KB_WARN("no factory registered for plugin type '{}'.", type.value());
				} else {
					return it->second;
				}
			} else {
				KB_WARN("missing 'type', 'lib' and 'module' key in plugin config.");
			}
			return {};
		}

		std::string
		getPluginID(const std::shared_ptr<PluginFactory<T>> &factory, const boost::property_tree::ptree &config) {
			auto name = config.get_optional<std::string>("name");
			if (name.has_value()) {
				return name.value();
			} else {
				return std::string(factory->name()) + std::to_string(pluginIndex_++);
			}
		}

		void removePlugin(const std::shared_ptr<NamedPlugin<T>> &namedPlugin) {
			pluginPool_.erase(namedPlugin->name());
		}
	};
}

#endif //KNOWROB_PLUGIN_MANAGER_H_
