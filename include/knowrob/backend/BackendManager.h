/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BACKEND_MANAGER_H_
#define KNOWROB_BACKEND_MANAGER_H_

#include <map>
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/plugins/PluginManager.h"
#include "QueryableBackend.h"

namespace knowrob {
	/**
	 * Manages a set of available backend subsystems.
	 */
	class BackendManager : public PluginManager<DataBackend> {
	public:
		/**
		 * @param kb the knowledge base this manager is associated with.
		 */
		explicit BackendManager(const std::shared_ptr<Vocabulary> &vocabulary);

		/**
		 * @return the vocabulary associated with this manager.
		 */
		auto &vocabulary() const { return vocabulary_; }

		/**
		 * @return map of all persistent backends defined by this manager.
		 */
		const auto &persistent() const { return persistent_; }

		/**
		 * @return map of all queryable backends defined by this manager.
		 */
		const auto &queryable() const { return queryable_; }

		/**
		 * Add a backend to this manager.
		 * @param backend a defined backend.
		 */
		void addPlugin(const std::shared_ptr<NamedBackend> &backend);

		// override PluginManager
		std::shared_ptr<NamedBackend> loadPlugin(const boost::property_tree::ptree &config) override;

		// override PluginManager
		std::shared_ptr<NamedBackend> addPlugin(std::string_view reasonerID, const DataBackendPtr &backend) override;

	private:
		std::shared_ptr<Vocabulary> vocabulary_;
		std::map<std::string, QueryableBackendPtr> persistent_;
		std::map<std::string, QueryableBackendPtr> queryable_;

		void initBackend(const std::shared_ptr<NamedBackend> &definedKG);
	};
}

// a macro for static registration of a knowledge graph type.
// knowledge graph types registered with this macro are builtin knowledge graphs that are not
// loaded from a plugin.
#define KNOWROB_BUILTIN_BACKEND(Name, Type) class Type ## _Registration{ static bool isRegistered; }; \
        bool Type ## _Registration::isRegistered = knowrob::PluginManager<knowrob::DataBackend>::addFactory<Type>(Name);

#endif //KNOWROB_BACKEND_MANAGER_H_
