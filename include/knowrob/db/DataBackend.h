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
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/reasoner/ReasonerConfig.h"
#include "knowrob/ThreadPool.h"
#include "DataSourceHandler.h"

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
		virtual bool initializeBackend(const ReasonerConfig &config) = 0;

		/**
		 * @return the vocabulary of this backend.
		 */
		const auto &vocabulary() const { return vocabulary_; }

		/**
		 * @return the import hierarchy between named graphs.
		 */
		const auto &importHierarchy() const { return importHierarchy_; }

		void setVocabulary(std::shared_ptr<Vocabulary> vocabulary) { vocabulary_ = std::move(vocabulary); }

		void setImportHierarchy(std::shared_ptr<ImportHierarchy> importHierarchy) {
			importHierarchy_ = std::move(importHierarchy);
		}

	protected:
		std::shared_ptr<Vocabulary> vocabulary_;
		std::shared_ptr<ImportHierarchy> importHierarchy_;
		BackendFeatures features_;

		void enableFeature(BackendFeature feature) { features_ = features_ | feature; }
	};

	using DataBackendPtr = std::shared_ptr<DataBackend>;

} // knowrob

#endif //KNOWROB_DATA_BACKEND_H
