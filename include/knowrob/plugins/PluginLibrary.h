/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PLUGIN_LIBRARY_H_
#define KNOWROB_PLUGIN_LIBRARY_H_

#include <string>
#include <memory>
#include <dlfcn.h>
#include "knowrob/plugins/PluginFactory.h"

namespace knowrob {
	/**
	 * A factory that uses a shared library for creation of plugin instances.
	 */
	template<class T>
	class PluginLibrary : public PluginFactory<T> {
	public:
		/**
		 * @param dllPath the name or path of the shared library.
		 */
		explicit PluginLibrary(std::string_view dllPath)
				: handle_(nullptr),
				  create_(nullptr),
				  get_name_(nullptr),
				  dllPath_(dllPath) {
		}

		~PluginLibrary() override {
			if (handle_) {
				dlclose(handle_);
				handle_ = nullptr;
			}
		}

		/**
		 * Cannot be copy-assigned.
		 */
		PluginLibrary(const PluginLibrary &) = delete;

		/**
		 * @return true if the shared library was loaded successfully.
		 */
		bool isLoaded() {
			return (create_ != nullptr && get_name_ != nullptr);
		}

		/**
		 * Try loading the shared library from filesystem.
		 * Note that, on unix-based systems, the LD_LIBRARY_PATH environment
		 * variable is used to locate the library.
		 * @return true on success.
		 */
		bool loadDLL() {
			handle_ = dlopen(dllPath_.c_str(), RTLD_LAZY);
			if (handle_ != nullptr) {
				create_ = (std::shared_ptr<T> (*)()) dlsym(handle_, "knowrob_createPlugin");
				get_name_ = (char *(*)()) dlsym(handle_, "knowrob_getPluginName");
				return isLoaded();
			} else {
				return false;
			}
		}

		// Override PluginFactory
		std::shared_ptr<NamedPlugin<T>> create(std::string_view pluginID) override {
			return std::make_shared<NamedPlugin<T>>(pluginID, create_());
		}

		// Override PluginFactory
		std::string_view name() const override { return name_; };

	protected:
		const std::string dllPath_;
		std::string name_;
		// handle of opened library
		void *handle_;

		// a factory function used to create new instances of a backend.
		std::shared_ptr<T> (*create_)();

		// a function that returns the name of the plugin
		char *(*get_name_)();
	};
}

#endif //KNOWROB_PLUGIN_LIBRARY_H_
