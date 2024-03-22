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
#include "knowrob/backend/Backend.h"
#include "knowrob/triples/SPARQLQuery.h"
#include "RedlandURI.h"
#include "knowrob/backend/SPARQLBackend.h"

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
	 * Redland hash types used by the "hashes" storage type.
	 */
	enum class RedlandHashType {
		// The default hash type.
		MEMORY,
		// The Berkeley DB hash type.
		BDB
	};

	/**
	 * Interface for a Redland model.
	 * A redland model is a triple store that can be queried with SPARQL.
	 * It can interface with different storage types, such as in-memory, MySQL, PostgreSQL, and SQLite,
	 * and can load and save triples in different formats, such as RDF/XML, Turtle, and N-Triples.
	 * It can further interface with SPARQL endpoints.
	 */
	class RedlandModel : public SPARQLBackend {
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
		 * Set the "hash-type" parameter of the storage.
		 * @param hashType the hash type.
		 */
		void setStorageHashType(RedlandHashType hashType);

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
		 * @param dir the directory for the storage.
		 */
		void setStorageDirectory(std::string_view dir = ".");

		/**
		 * Set the storage type to "hashes".
		 * @param storageType the storage type.
		 * @param dir the directory for the hashes storage.
		 * @see https://librdf.org/docs/api/redland-storage-module-hashes.html
		 */
		void setHashesStorage(RedlandHashType hashType, std::string_view dir = ".");

		/**
		 * Set the storage type to "memory".
		 * @see https://librdf.org/docs/api/redland-storage-module-memory.html
		 */
		void setMemoryStorage();

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
		bool initializeBackend(const PropertyTree &config) override;

		/**
		 * Load an URI into the model.
		 * Note that in case of SPARQL endpoints, this does not mean al data is copied into the model.
		 * @param uri the uri to load.
		 * @param tripleFormat the format of the triples.
		 * @return true if the uri was loaded successfully.
		 */
		bool load(const URI &uri, semweb::TripleFormat tripleFormat);

		// override SPARQLBackend
		bool sparql(std::string_view queryString, const BindingsHandler &callback) const override;

		// override DataBackend
		bool insertOne(const FramedTriple &triple) override;

		// override DataBackend
		bool insertAll(const TripleContainerPtr &triples) override;

		// override DataBackend
		bool removeOne(const FramedTriple &triple) override;

		// override DataBackend
		bool removeAll(const TripleContainerPtr &triples) override;

		// override DataBackend
		bool removeAllWithOrigin(std::string_view origin) override;

		// Override QueryableBackend
		bool isPersistent() const override;

		// Override QueryableBackend
		bool contains(const FramedTriple &triple) override;

		// Override QueryableBackend
		void batch(const TripleHandler &callback) const override;

		// Override QueryableBackend
		void match(const FramedTriplePattern &query, const TripleVisitor &visitor) override;

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
		std::optional<std::string> storageDir_;
		std::optional<RedlandHashType> hashType_;
		RedlandStorageType storageType_;
		std::string storageOptions_;

		std::string getStorageOptions() const;

		void knowrobToRaptor(const FramedTriple &triple, raptor_statement *raptorTriple);

		void knowrobToRaptor(const FramedTriplePattern &pat, raptor_statement *raptorTriple);

		raptor_term *knowrobToRaptor(const TermPtr &term);

		librdf_node *getContextNode(std::string_view origin);

		librdf_node *getContextNode(const FramedTriple &triple);

	private:
		void finalize();
	};

} // knowrob

#endif //KNOWROB_REDLAND_MODEL_H
