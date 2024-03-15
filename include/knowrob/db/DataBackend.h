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
	 * A data backend is a component that stores extensional data.
	 * Note that data backends do not do reification internally --
	 * reification is only handled via the transaction controller.
	 * Meaning that data backends without support for triple context
	 * might just ignore any contextual parameters handed to them.
	 */
	class DataBackend : public DataSourceHandler {
	public:
		DataBackend() = default;

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
		virtual bool insertAll(const semweb::TripleContainerPtr &triples) = 0;

		/**
		 * Delete the first matching statement from this backend.
		 * @param triple a triple.
		 */
		virtual bool removeOne(const FramedTriple &triple) = 0;

		/**
		 * Delete all matching statements from this backend.
		 * @param triples a set of triples.
		 */
		virtual bool removeAll(const semweb::TripleContainerPtr &triples) = 0;

		/**
		 * Delete all statements with a given origin from this backend.
		 * @param origin the origin of the statements to be deleted.
		 */
		virtual bool removeAllWithOrigin(std::string_view origin) = 0;

		/**
		 * Check if a triple may have additional context information that can be
		 * stored in this backend without the use of reified statements.
		 * If this is not the case, statements with additional context information
		 * first need to be reified before they can be handled by this backend.
		 * @return true if this backend can store triple context directly
		 */
		virtual bool canStoreTripleContext() const { return false; }

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

		void setVocabulary(std::shared_ptr<semweb::Vocabulary> vocabulary) { vocabulary_ = std::move(vocabulary); }

		void setImportHierarchy(std::shared_ptr<semweb::ImportHierarchy> importHierarchy) {
			importHierarchy_ = std::move(importHierarchy);
		}

	protected:
		std::shared_ptr<semweb::Vocabulary> vocabulary_;
		std::shared_ptr<semweb::ImportHierarchy> importHierarchy_;
	};

	using DataBackendPtr = std::shared_ptr<DataBackend>;

} // knowrob

#endif //KNOWROB_DATA_BACKEND_H
