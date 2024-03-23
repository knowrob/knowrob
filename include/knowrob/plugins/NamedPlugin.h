/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_NAMED_PLUGIN_H_
#define KNOWROB_NAMED_PLUGIN_H_

#include <string>
#include <memory>

namespace knowrob {
	/**
	 * A plugin with a name.
	 */
	template<class T>
	class NamedPlugin {
	public:
		/**
		 * @param name the name of the plugin
		 * @param reasoner the plugin instance
		 */
		NamedPlugin(std::string_view name, const std::shared_ptr<T> &plugin)
				: name_(name), plugin_(plugin) {}

		/**
		 * @return the plugin instance
		 */
		const std::shared_ptr<T> &operator()() const { return plugin_; }

		/**
		 * @return the plugin instance
		 */
		const std::shared_ptr<T> &value() const { return plugin_; }

		/**
		 * @return the plugin name.
		 */
		std::string_view name() const { return name_; }

	protected:
		const std::string name_;
		const std::shared_ptr<T> plugin_;
	};
}

#endif //KNOWROB_NAMED_PLUGIN_H_
