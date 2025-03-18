/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/reasoner/ReasonerError.h"

using namespace knowrob;

ReasonerManager::ReasonerManager(KnowledgeBase *kb, const std::shared_ptr<StorageManager> &backendManager)
		: PluginManager(),
		  kb_(kb),
		  backendManager_(backendManager) {
}

ReasonerManager::~ReasonerManager() {
	std::lock_guard<std::mutex> scoped_lock(staticMutex_);
	for (auto &x: pluginPool_) {
		x.second->value()->unload();
		// make sure reasoner does not interact with the manager anymore
		x.second->value()->setReasonerManager(nullptr);
	}
	pluginPool_.clear();
}

void ReasonerManager::setReasonerStorage(const std::shared_ptr<NamedPlugin<Reasoner>> &plugin,
										 const std::shared_ptr<Storage> &dataBackend) {
	if (plugin->language() == PluginLanguage::PYTHON) {
		py::gil_lock acquire;
		plugin->value()->setStorage(dataBackend);
	} else {
		plugin->value()->setStorage(dataBackend);
	}
	reasonerBackends_[plugin->value()->reasonerName()->stringForm()] = dataBackend;
}

std::shared_ptr<Storage> ReasonerManager::getReasonerStorage(const std::shared_ptr<NamedReasoner> &reasoner) {
	auto it = reasonerBackends_.find(reasoner->name());
	if (it != reasonerBackends_.end()) {
		return it->second;
	} else {
		return nullptr;
	}
}

std::vector<DefiningReasoner> ReasonerManager::findDefiningReasoner(const PredicateIndicator &indicator) const {
	std::vector<DefiningReasoner> reasoners;

	if (indicator.arity() == 1) {
		// unary predicates can be RDF class expressions.
		// in this case we need to check the class hierarchy.
		auto rdfClass = kb_->vocabulary()->getDefinedClass(indicator.functor()->stringForm());
		if (rdfClass) {
			for (auto &x: goalDriven_) {
				for (auto &definedClassIndicator : x.second->definedClasses()) {
					auto definedClass = kb_->vocabulary()->getDefinedClass(definedClassIndicator.functor()->stringForm());
					if(definedClass && definedClass->isSubClassOf(rdfClass)) {
						reasoners.push_back({x.second, definedClass->iriAtom()});
						break;
					}
				}
			}
			return reasoners;
		}
	}

	if (indicator.arity() == 2) {
		// binary predicates can be RDF properties.
		// in this case we need to check the property hierarchy.
		auto rdfProperty = kb_->vocabulary()->getDefinedProperty(indicator.functor()->stringForm());
		if (rdfProperty) {
			for (auto &x: goalDriven_) {
				for (auto &definedPropertyIndicator : x.second->definedRelations()) {
					auto definedProperty = kb_->vocabulary()->getDefinedProperty(definedPropertyIndicator.functor()->stringForm());
                    if ( definedProperty && definedProperty->isSubPropertyOf(rdfProperty) ) {
						reasoners.push_back({x.second, definedProperty->iriAtom()});
						break;
					}
				}
			}
			return reasoners;
		}
	}

	for (auto &x: goalDriven_) {
		if (x.second->isRelationDefined(indicator) || (indicator.arity()==1 && x.second->isClassDefined(indicator.functor()->stringForm()))) {
			reasoners.push_back({x.second, indicator.functor()});
		}
	}
	return reasoners;
}

std::shared_ptr<NamedReasoner> ReasonerManager::loadPlugin(const boost::property_tree::ptree &config) {
	// get a reasoner factory
	std::shared_ptr<ReasonerFactory> factory = findFactory(config);
	// make sure factory was found above
	if (!factory) throw ReasonerError("failed to load a reasoner.");
	// create a reasoner id, or use name property
	std::string reasonerID = getPluginID(factory, config);
	KB_INFO("Using reasoner `{}` with type `{}`.", reasonerID, factory->name());

	// create a new reasoner instance
	auto reasoner = factory->create(reasonerID);
	// reasoner need to have a reference to the reasoner manager such that
	// predicates can be defined that interact with the KB
	reasoner->value()->setReasonerManager(this);
	reasoner->value()->setReasonerName(reasonerID);

	auto backendName = config.get_optional<std::string>("data-backend");
	if (backendName.has_value()) {
		auto definedBackend = backendManager_->getPluginWithID(backendName.value());
		if (definedBackend) {
			setReasonerStorage(reasoner, definedBackend->value());
		} else {
			throw ReasonerError("Reasoner `{}` refers to unknown data-backend `{}`.", reasonerID, backendName.value());
		}
	} else {
		// check if reasoner implements DataBackend interface
		auto backend = std::dynamic_pointer_cast<Storage>(reasoner->value());
		if (backend) {
			setReasonerStorage(reasoner, backend);
			backendManager_->addPlugin(reasonerID, reasoner->language(), backend);
		} else {
			throw ReasonerError("Reasoner `{}` has no 'data-backend' configured.", reasonerID);
		}
	}
	auto definedReasoner = addPlugin(reasonerID, reasoner->language(), reasoner->value());

	PropertyTree pluginConfig(std::make_shared<boost::property_tree::ptree>(config));
	if (!initializeReasoner(reasoner, pluginConfig)) {
		KB_WARN("Reasoner `{}` failed to loadConfig.", reasonerID);
	} else {
		// load the reasoner-specific data sources.
		for (auto &dataSource: pluginConfig.dataSources()) {
			if (!reasoner->value()->loadDataSource(dataSource)) {
				KB_WARN("Reasoner `{}` failed to load data source {}.", reasonerID, dataSource->uri());
			}
		}
	}

	return definedReasoner;
}

std::shared_ptr<NamedReasoner>
ReasonerManager::addPlugin(std::string_view reasonerID, PluginLanguage language, const std::shared_ptr<Reasoner> &reasoner) {
	if (pluginPool_.find(reasonerID) != pluginPool_.end()) {
		KB_WARN("overwriting reasoner with name '{}'", reasonerID);
	}
	auto managedReasoner = std::make_shared<NamedReasoner>(reasonerID, language, reasoner);
	pluginPool_.emplace(managedReasoner->name(), managedReasoner);
	reasoner->setReasonerManager(this);
	reasoner->setReasonerName(reasonerID);
	initPlugin(managedReasoner);
	// indicate that the origin `reasonerID` is a reasoner, and thus belongs to the session
	backendManager_->vocabulary()->importHierarchy()->addDirectImport(
			backendManager_->vocabulary()->importHierarchy()->ORIGIN_REASONER, reasonerID);

	// check if reasoner implements DataBackend interface
	auto backend = std::dynamic_pointer_cast<Storage>(reasoner);
	if (backend) {
		setReasonerStorage(managedReasoner, backend);
		backendManager_->addPlugin(reasonerID, language, backend);
	}

	return managedReasoner;
}

bool ReasonerManager::initializeReasoner(const std::shared_ptr<NamedReasoner> &namedReasoner, PropertyTree &config) {
	if (namedReasoner->language() == PluginLanguage::PYTHON) {
		py::gil_lock acquire;
		return namedReasoner->value()->initializeReasoner(config);
	} else {
		return namedReasoner->value()->initializeReasoner(config);
	}
}

void ReasonerManager::initPlugin(const std::shared_ptr<NamedReasoner> &namedReasoner) {
	// initialize the reasoner language type (entering python code requires special treatment)
	namedReasoner->value()->setReasonerLanguage(namedReasoner->language());
	// check if the reasoner is data-driven
	auto dataDriven = std::dynamic_pointer_cast<DataDrivenReasoner>(namedReasoner->value());
	if (dataDriven) {
		KB_INFO("Using data-driven reasoner with id '{}'.", namedReasoner->name());
		dataDriven_[namedReasoner->name()] = dataDriven;
	}
	// check if the reasoner is goal-driven
	auto goalDriven = std::dynamic_pointer_cast<GoalDrivenReasoner>(namedReasoner->value());
	if (goalDriven) {
		KB_INFO("Using goal-driven reasoner with id '{}'.", namedReasoner->name());
		goalDriven_[namedReasoner->name()] = goalDriven;
	}
}

TokenBufferPtr ReasonerManager::evaluateQuery(
		const GoalDrivenReasonerPtr &reasoner,
		const std::vector<FirstOrderLiteralPtr> &literals,
		const QueryContextPtr &ctx) {
	auto reasonerRunner = std::make_shared<ReasonerRunner>();
	reasonerRunner->reasoner = reasoner;
	if(literals.size()>1) {
		auto conjunction = std::make_shared<SimpleConjunction>(literals);
		reasonerRunner->query = std::make_shared<Goal>(conjunction, ctx);
	} else if (literals.size() == 1) {
		reasonerRunner->query = std::make_shared<Goal>(literals[0], ctx);
	} else {
		throw ReasonerError("Reasoner {} received an empty query.", *reasoner->reasonerName());
	}
	KB_DEBUG("Evaluating query `{}` with reasoner \"{}\"",
			*reasonerRunner->query->formula(), *reasoner->reasonerName());
	// avoid that the exception handler keeps a reference to the reasonerRunner as
	// this would prevent the runner from being destroyed.
	auto reasonerRunnerPtr = reasonerRunner.get();
	// run reasoner in a thread
	DefaultThreadPool()->pushWork(
			reasonerRunner,
			[reasonerRunnerPtr](const std::exception &exc) {
				KB_ERROR("Reasoner {} produced an error in query evaluation. {} [{}]",
						 *reasonerRunnerPtr->reasoner->reasonerName(), exc.what(), *reasonerRunnerPtr->query->formula());
				reasonerRunnerPtr->query->finish();
			});
	// return the (incomplete) answer buffer
	return reasonerRunner->query->answerBuffer();
}
