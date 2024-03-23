/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_MANAGER_H_
#define KNOWROB_REASONER_MANAGER_H_

#include "knowrob/plugins/PluginManager.h"
#include "knowrob/backend/BackendManager.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/reasoner/GoalDrivenReasoner.h"
#include "knowrob/reasoner/DataDrivenReasoner.h"

namespace knowrob {
	/**
	 * Manages a set of available reasoning subsystems.
	 */
	class ReasonerManager : public PluginManager<Reasoner> {
	public:
		/**
		 * Create a new reasoner manager.
		 * @param kb the knowledge base associated with this manager.
		 * @param backendManager the backend manager associated with this manager.
		 */
		ReasonerManager(KnowledgeBase *kb, const std::shared_ptr<BackendManager> &backendManager);

		~ReasonerManager();

		/**
		 * @return the knowledge base associated with this manager.
		 */
		auto kb() const { return kb_; }

		/**
		 * @return the goal-driven reasoners defined by this manager.
		 */
		auto &goalDriven() const { return goalDriven_; }

		/**
		 * @return the data-driven reasoners defined by this manager.
		 */
		auto &dataDriven() const { return dataDriven_; }

		/**
		 * Return the backend associated with a reasoner if any.
		 * @param reasoner a defined reasoner.
		 * @return a backend or a null reference.
		 */
		std::shared_ptr<DataBackend> getReasonerBackend(const std::shared_ptr<NamedReasoner> &reasoner);

		// override PluginManager
		std::shared_ptr<NamedReasoner> loadPlugin(const boost::property_tree::ptree &config) override;

		// override PluginManager
		std::shared_ptr<NamedReasoner>
		addPlugin(std::string_view reasonerID, const std::shared_ptr<Reasoner> &reasoner) override;

	private:
		KnowledgeBase *kb_;
		std::shared_ptr<BackendManager> backendManager_;
		// maps reasoner to their backends
		std::map<std::string_view, DataBackendPtr, std::less<>> reasonerBackends_;
		std::map<std::string_view, DataDrivenReasonerPtr> dataDriven_;
		std::map<std::string_view, GoalDrivenReasonerPtr> goalDriven_;

		void setDataBackend(const ReasonerPtr &reasoner, const std::shared_ptr<DataBackend> &dataBackend);

		void initPlugin(const std::shared_ptr<NamedReasoner> &namedReasoner);
	};
}

// a macro for static registration of a reasoner type.
// reasoner types registered with this macro are builtin reasoners that are not
// loaded from a plugin.
#define KNOWROB_BUILTIN_REASONER(Name, Type) class Type ## _Registration{ static bool isRegistered; }; \
        bool Type ## _Registration::isRegistered = knowrob::PluginManager<knowrob::Reasoner>::addFactory<Type>(Name);

#endif //KNOWROB_REASONER_MANAGER_H_
