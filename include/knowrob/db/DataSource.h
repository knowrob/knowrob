/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_SOURCES_H_
#define KNOWROB_DATA_SOURCES_H_

#include <string>
#include "knowrob/URI.h"

namespace knowrob {
	/**
	 * Some data source types that receive special handling in the knowledge base.
	 */
	enum class DataSourceType {
		ONTOLOGY,
		SPARQL,
		UNSPECIFIED
	};

	/**
	 * A data source is a source of data that can be loaded into a subsystem of the
	 * knowledge base. This can be a file, a database, a web service, etc.
	 */
	class DataSource {
	public:
		/**
		 * @param uri URI of the data source.
		 * @param format string identifier of the data format.
		 */
		DataSource(URI uri, std::string_view format);

		/**
		 * @return URI of this data source.
		 */
		const auto &uri() const { return uri_(); }

		/**
		 * @return string identifier of the data format.
		 */
		const auto &path() const { return uri_.path(); }

		/**
		 * @return string identifier of the data format.
		 */
		const auto &format() const { return format_; }

		/**
		 * @return the type of the data source.
		 */
		virtual DataSourceType type() const { return DataSourceType::UNSPECIFIED; }

		/**
		 * @return the name of the data source.
		 */
		auto name() const { return getNameFromURI(uri()); }

		/**
		 * @return the version of the data source.
		 */
		auto version() const { return getVersionFromURI(uri()); }

		/**
		 * Ontologies are loaded into named sub-graphs of the knowledge graph.
		 * The name is generated from the URI in case of loading RDF files.
		 * @param uriString A URI pointing to a RDF ontology.
		 * @return a graph name for the ontology
		 */
		static std::string getNameFromURI(const std::string &uriString);

		/**
		 * Extract a version string from an ontology URI.
		 * In case the URI points to a local file, the modification time of the file
		 * is used as version.
		 * For other URIs it is attempted to extract version information from the URI string,
		 * if this fails, then the current day is used as a version string.
		 * @param uriString A URI pointing to a RDF ontology.
		 * @return a version string
		 */
		static std::string getVersionFromURI(const std::string &uriString);

		/**
		 * @param versionString a string
		 * @return true if versionString is a valid version string
		 */
		static bool isVersionString(const std::string &versionString);

	protected:
		std::string format_;
		URI uri_;
	};

	// alias
	using DataSourcePtr = std::shared_ptr<DataSource>;
}

#endif //KNOWROB_DATA_SOURCES_H_
