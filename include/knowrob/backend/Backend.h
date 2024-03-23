/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_BACKEND_H
#define KNOWROB_DATA_BACKEND_H

#include <boost/property_tree/ptree.hpp>
#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/triples/FramedTriple.h"
#include "knowrob/triples/TripleContainer.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/PropertyTree.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/sources/DataSourceHandler.h"
#include "knowrob/plugins/PluginFactory.h"
#include "knowrob/plugins/TypedPluginFactory.h"

namespace knowrob {
	/**
	 * A flag that indicates whether a SPARQL feature is supported or not.
	 */
	enum class BackendFeature : std::uint8_t {
		// A flag that indicates that nothing special is supported.
		NothingSpecial = 1ul << 0,
		// A flag that indicates that re-assignment of variables within query pipelines is supported.
		ReAssignment = 1ul << 2,
		// A flag that indicates that triple context is supported.
		// If this flag is not set, statements with additional context information first must be reified
		// before they can be handled by this backend.
		TripleContext = 1ul << 3
	};
	using BackendFeatures = BackendFeature;

	/**
	 * Compute the bitwise OR of two SPARQL flags.
	 * @param a a flag.
	 * @param b a flag.
	 * @return the bitwise OR of a and b.
	 */
	BackendFeature operator|(BackendFeature a, BackendFeature b);

	/**
	 * Compute the bitwise AND of two SPARQL flags.
	 * @param a a flag.
	 * @param b a flag.
	 * @return the bitwise AND of a and b.
	 */
	bool operator&(BackendFeature a, BackendFeature b);

	/**
	 * A data backend is a component that stores extensional data.
	 * Note that data backends do not do reification internally --
	 * reification is only handled via BackendInterface.
	 * Meaning that data backends without support for triple context
	 * can just ignore any contextual parameters handed to them.
	 */
	class DataBackend : public DataSourceHandler {
	public:
		explicit DataBackend(BackendFeatures features = BackendFeature::NothingSpecial) : features_(features) {}

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
		bool supports(BackendFeature feature) const { return features_ & feature; }

		/**
		 * Add an assertion to this backend.
		 * @param triple a triple.
		 * @return true on success
		 */
		virtual bool insertOne(const FramedTriple &triple) = 0;

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
		virtual bool removeOne(const FramedTriple &triple) = 0;

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
		virtual bool initializeBackend(const PropertyTree &ptree) = 0;

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
		BackendFeatures features_;

		void enableFeature(BackendFeature feature) { features_ = features_ | feature; }
	};

	using DataBackendPtr = std::shared_ptr<DataBackend>;
	using BackendFactory = PluginFactory<DataBackend>;
	using NamedBackend = NamedPlugin<DataBackend>;

} // knowrob

/**
 * Define a data backend plugin.
 * The macro generates two functions that are used as entry points for
 * loading the plugin.
 * First, a factory function is defined that creates instances of @classType.
 * This will only work when @classType has a single argument constructor that
 * accepts a string as argument (the KG instance ID).
 * Second, a function is generated that exposes the plugin name.
 * @param classType the type of the backend, must be a subclass of DataBackend
 * @param pluginName a plugin identifier, e.g. the name of the backend type.
 */
#define KNOWROB_BACKEND_PLUGIN(classType, pluginName) extern "C" { \
        std::shared_ptr<knowrob::DataBackend> knowrob_createPlugin(const std::string &pluginID) \
            { return std::make_shared<classType>(); } \
        const char* knowrob_getPluginName() { return pluginName; } }

#endif //KNOWROB_DATA_BACKEND_H
