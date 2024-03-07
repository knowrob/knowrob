/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REDLAND_MODEL_H
#define KNOWROB_REDLAND_MODEL_H

#include <redland.h>
#include "knowrob/triples/TripleContainer.h"
#include "knowrob/URI.h"
#include "knowrob/triples/TripleFormat.h"
#include "DataBackend.h"
#include "knowrob/queries/SPARQLQuery.h"
#include "RedlandURI.h"
#include "QueryableBackend.h"

namespace knowrob {
	/**
	 * Redland storage types.
	 * @see https://librdf.org/docs/api/redland-storage-modules.html
	 */
	enum class RedlandStorageType {
		// This module is always present (cannot be removed) and provides a simple and fast in-memory store with no persistence.
		MEMORY,
		// This module is always present (cannot be removed) and provides indexed storage using Redland Triple stores
		HASHES,
		// This module is compiled in when MySQL 3 or 4 is available.
		MYSQL,
		// This module is based on the MySQL store and is compiled in when PostgreSQL is available.
		POSTGRESQL,
		// This module provides storage via the SQLite relational database when available and supports SQLite V2 and V3.
		SQLITE
	};

	/**
	 * Interface for a Redland model.
	 * A redland model is a triple store that can be queried with SPARQL.
	 * It can interface with different storage types, such as in-memory, MySQL, PostgreSQL, and SQLite,
	 * and can load and save triples in different formats, such as RDF/XML, Turtle, and N-Triples.
	 * It can further interface with SPARQL endpoints.
	 */
	class RedlandModel : public QueryableBackend {
	public:
		static constexpr std::string_view QUERY_LANGUAGE_SPARQL = "sparql";

		RedlandModel();

		~RedlandModel();

		/**
		 * Set the storage type.
		 * @param storageType the storage type.
		 */
		void setStorageType(RedlandStorageType storageType);

		/**
		 * @param host the host of the database.
		 */
		void setHost(std::string_view host);

		/**
		 * @param database the database name.
		 */
		void setDatabase(std::string_view database);

		/**
		 * @param user the user name for the database.
		 */
		void setUser(std::string_view user);

		/**
		 * @param password the password for the database.
		 */
		void setPassword(std::string_view password);

		/**
		 * If not set, the model will create its own.
		 * @param world a redland world.
		 */
		void setRaptorWorld(librdf_world *world);

		/**
		 * If set assume triples in this backend have this as a default origin.
		 * @param origin the origin of the triples.
		 */
		void setOrigin(std::string_view origin) { origin_ = origin; }

		/**
		 * @return the redland storage type of the model.
		 */
		auto storageType() const { return storageType_; }

		/**
		 * @return true if the model is initialized.
		 */
		bool isInitialized() const;

		/**
		 * Initialize the backend.
		 * @return true if the backend was initialized successfully.
		 */
		bool initializeBackend();

		// override DataBackend
		bool initializeBackend(const ReasonerConfig &config) override;

		/**
		 * Load an URI into the model.
		 * Note that in case of SPARQL endpoints, this does not mean al data is copied into the model.
		 * @param uri the uri to load.
		 * @param tripleFormat the format of the triples.
		 * @return true if the uri was loaded successfully.
		 */
		bool load(const URI &uri, semweb::TripleFormat tripleFormat);

		/**
		 * Run a SPARQL query on the model.
		 * @param queryString the query to run.
		 * @param callback the callback to handle the results.
		 * @return true if the query was successful.
		 */
		bool sparql(std::string_view queryString, const FramedBindingsHandler &callback) const;

		/**
		 * Run a SPARQL query on the model.
		 * @param query the query to run.
		 * @param callback the callback to handle the results.
		 * @return true if the query was successful.
		 */
		bool query(const SPARQLQuery &query, const FramedBindingsHandler &callback) const;

		// override DataBackend
		bool insertOne(const FramedTriple &triple) override;

		// override DataBackend
		bool insertAll(const semweb::TripleContainerPtr &triples) override;

		// override DataBackend
		bool removeOne(const FramedTriple &triple) override;

		// override DataBackend
		bool removeAll(const semweb::TripleContainerPtr &triples) override;

		// override DataBackend
		bool removeAllWithOrigin(std::string_view origin) override;

		// Override QueryableBackend
		bool isPersistent() const override;

		// Override QueryableBackend
		bool contains(const FramedTriple &triple) override;

		// Override QueryableBackend
		void foreach(const semweb::TripleVisitor &visitor) const override;

		// Override QueryableBackend
		void batch(const semweb::TripleHandler &callback) const override;

		// Override QueryableBackend
		void match(const FramedTriplePattern &query, const semweb::TripleVisitor &visitor) override;

		// Override QueryableBackend
		void query(const ConjunctiveQueryPtr &query, const FramedBindingsHandler &callback) override;

		// Override QueryableBackend
		void count(const ResourceCounter &callback) const override;

	protected:
		librdf_world *ownedWorld_;
		librdf_world *world_;
		librdf_model *model_;
		librdf_storage *storage_;

		// the uris are created within the context of a librdf_world, so
		// they cannot be defined statically.
		RedlandURI xsdURIs_[static_cast<int>(XSDType::LAST)];

		RedlandURI &xsdURI(XSDType xsdType);

		std::map<std::string, librdf_node *, std::less<>> contextNodes_;

		std::string storageName_;
		// if set when querying, ignore context nodes and use this origin
		std::optional<std::string> origin_;
		std::optional<std::string> host_;
		std::optional<std::string> port_;
		std::optional<std::string> database_;
		std::optional<std::string> user_;
		std::optional<std::string> password_;
		RedlandStorageType storageType_;
		std::string storageOptions_;

		std::string getStorageOptions() const;

		void knowrobToRaptor(const FramedTriple &triple, raptor_statement *raptorTriple);

		void knowrobToRaptor(const FramedTriplePattern &pat, raptor_statement *raptorTriple);

		raptor_term *knowrobToRaptor(const TermPtr &term);

		librdf_node *getContextNode(std::string_view origin);

		librdf_node *getContextNode(const FramedTriple &triple);

		void batch(const semweb::TripleHandler &callback, uint32_t batchSize) const;

	private:
		void finalize();
	};

} // knowrob

#endif //KNOWROB_REDLAND_MODEL_H
