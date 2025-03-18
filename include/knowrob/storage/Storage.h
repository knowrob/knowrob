/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_STORAGE_H
#define KNOWROB_STORAGE_H

#include <boost/property_tree/ptree.hpp>
#include "knowrob/semweb/TriplePattern.h"
#include "knowrob/semweb/Triple.h"
#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/PropertyTree.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/DataSourceHandler.h"
#include "knowrob/plugins/PluginFactory.h"
#include "knowrob/plugins/TypedPluginFactory.h"

namespace knowrob {
	/**
	 * A flag that indicates whether a SPARQL feature is supported or not.
	 */
	enum class StorageFeature : std::uint8_t {
		// A flag that indicates that nothing special is supported.
		NothingSpecial = 1ul << 0,
		// A flag that indicates that re-assignment of variables within query pipelines is supported.
		ReAssignment = 1ul << 2,
		// A flag that indicates that triple context is supported.
		// If this flag is not set, statements with additional context information first must be reified
		// before they can be handled by this backend.
		TripleContext = 1ul << 3
	};
	using StorageFeatures = StorageFeature;

	/**
	 * Compute the bitwise OR of two SPARQL flags.
	 * @param a a flag.
	 * @param b a flag.
	 * @return the bitwise OR of a and b.
	 */
	StorageFeature operator|(StorageFeature a, StorageFeature b);

	/**
	 * Compute the bitwise AND of two SPARQL flags.
	 * @param a a flag.
	 * @param b a flag.
	 * @return the bitwise AND of a and b.
	 */
	bool operator&(StorageFeature a, StorageFeature b);

	/**
	 * A data backend is a component that stores extensional data.
	 * Note that data backends do not do reification internally --
	 * reification is only handled via BackendInterface.
	 * Meaning that data backends without support for triple context
	 * can just ignore any contextual parameters handed to them.
	 */
	class Storage : public DataSourceHandler {
	public:
		explicit Storage(StorageFeatures features = StorageFeature::NothingSpecial) : features_(features), storageLanguage_(PluginLanguage::CPP) {}

		virtual ~Storage() = default;

		/**
		 * @return the language of the storage plugin.
		 */
		PluginLanguage storageLanguage() const { return storageLanguage_; }

		/**
		 * @return the vocabulary of this backend.
		 */
		const auto &vocabulary() const { return vocabulary_; }

		/**
		 * Set the vocabulary of this backend.
		 * @param vocabulary a vocabulary.
		 */
		void setVocabulary(std::shared_ptr<Vocabulary> vocabulary) { vocabulary_ = std::move(vocabulary); }

		/**
		 * @return true if the backend supports re-assignment of variables within query pipelines.
		 */
		bool supports(StorageFeature feature) const { return features_ & feature; }

		/**
		 * Add an assertion to this backend.
		 * @param triple a triple.
		 * @return true on success
		 */
		virtual bool insertOne(const Triple &triple) = 0;

		/**
		 * Add assertions to this backend.
		 * @param triples a set of triples.
		 * @return true on success
		 */
		virtual bool insertAll(const TripleContainerPtr &triples) = 0;

		/**
		 * Delete the first matching statement from this backend.
		 * @param triple a triple.
		 */
		virtual bool removeOne(const Triple &triple) = 0;

		/**
		 * Delete all matching statements from this backend.
		 * @param triples a set of triples.
		 */
		virtual bool removeAll(const TripleContainerPtr &triples) = 0;

		/**
		 * Delete all statements with a given origin from this backend.
		 * @param origin the origin of the statements to be deleted.
		 */
		virtual bool removeAllWithOrigin(std::string_view origin) = 0;

		/**
		 * Initialize this backend from a property tree.
		 * @param config a property tree.
		 * @return true on success
		 */
		virtual bool initializeBackend(const PropertyTree &config) = 0;

		std::optional<std::string> getVersionOfOrigin(std::string_view origin) const {
			auto it = originVersions_.find(origin.data());
			if (it != originVersions_.end()) {
				return it->second;
			}
			return std::nullopt;
		}

		void setVersionOfOrigin(std::string_view origin, std::optional<std::string_view> version) {
			if (version) {
				originVersions_[origin.data()] = version.value().data();
			} else {
				originVersions_.erase(origin.data());
			}
		}

	protected:
		std::map<std::string, std::string> originVersions_;
		std::shared_ptr<Vocabulary> vocabulary_;
		StorageFeatures features_;
		PluginLanguage storageLanguage_;

		void enableFeature(StorageFeature feature) { features_ = features_ | feature; }

		void setStorageLanguage(PluginLanguage storageLanguage) { storageLanguage_ = storageLanguage; }

		friend class StorageManager;
	};

	using StoragePtr = std::shared_ptr<Storage>;
	using BackendFactory = PluginFactory<Storage>;
	using NamedBackend = NamedPlugin<Storage>;

} // knowrob

/**
 * Define a data backend plugin.
 * The macro generates two functions that are used as entry points for
 * loading the plugin.
 * First, a factory function is defined that creates instances of classType.
 * This will only work when classType has a single argument constructor that
 * accepts a string as argument (the KG instance ID).
 * Second, a function is generated that exposes the plugin name.
 * @param classType the type of the backend, must be a subclass of DataBackend
 * @param pluginName a plugin identifier, e.g. the name of the backend type.
 */
#define KNOWROB_STORAGE_PLUGIN(classType, pluginName) extern "C" { \
        std::shared_ptr<knowrob::Storage> knowrob_createPlugin(const std::string &pluginID) \
            { return std::make_shared<classType>(); } \
        const char* knowrob_getPluginName() { return pluginName; } }

#endif //KNOWROB_STORAGE_H
