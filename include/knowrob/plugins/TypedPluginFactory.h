/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TYPED_PLUGIN_FACTORY_H_
#define KNOWROB_TYPED_PLUGIN_FACTORY_H_

#include <string>
#include <memory>
#include "PluginFactory.h"

namespace knowrob {
	/**
	 * A plugin factory implementation for builtin types.
	 * @tparam T the type of plugin.
	 */
	template<class T, class Base>
	class TypedPluginFactory : public PluginFactory<Base> {
	public:
		/**
		 * @param name name of the plugin type for which the factory can create instances.
		 */
		explicit TypedPluginFactory(std::string_view name) : name_(name) {}

		// Override PluginFactory
		std::shared_ptr<NamedPlugin<Base>> create(std::string_view pluginID) override {
			return std::make_shared<NamedPlugin<Base>>(pluginID, std::make_shared<T>());
		}

		// Override PluginFactory
		std::string_view name() const override { return name_; }

	protected:
		std::string name_;
	};
}

#endif //KNOWROB_TYPED_PLUGIN_FACTORY_H_
