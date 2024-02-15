/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_PARSER_H
#define KNOWROB_ONTOLOGY_PARSER_H

#include <functional>
#include <string>
#include <raptor.h>
#include "knowrob/semweb/TripleContainer.h"
#include "knowrob/semweb/TripleFormat.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/semweb/GraphSelector.h"

namespace knowrob {
	/**
	 * A parser for RDF data.
	 */
	class OntologyParser {
	public:
		static const std::string NAME_TURTLE;
		static const std::string NAME_TRIG;
		static const std::string NAME_GRDDL;
		static const std::string NAME_NTRIPLES;
		static const std::string NAME_RDFXML;
		static const std::string NAME_RDFA;
		static const std::string NAME_RSS_TAG_SOUP;

		/**
		 * @param fileURI the URI of the file to parse.
		 * @param format the format of the file.
		 * @param batchSize the max size of each batch to use for parsing.
		 */
		OntologyParser(const std::string_view &fileURI, semweb::TripleFormat format, uint32_t batchSize = 1000);

		~OntologyParser();

		/**
		 * @param frame the graph selector to use for filtering triples.
		 */
		void setFrame(const GraphSelectorPtr &frame) { frame_ = frame; }

		/**
		 * @param origin the origin to use for triples.
		 */
		void setOrigin(std::string_view origin) { origin_ = origin; }

		/**
		 * @param blankPrefix the prefix to use for blank nodes.
		 */
		void setBlankPrefix(const std::string_view &blankPrefix) { blankPrefix_ = blankPrefix; }

		/**
		 * @param filter the filter to use for filtering triples.
		 */
		void setFilter(const semweb::TripleFilter &filter) { filter_ = filter; }

		/**
		 * @param callback the callback to use for handling triples.
		 * @return true if the parsing was successful, false otherwise.
		 */
		bool run(const semweb::TripleHandler &callback);

		/**
		 * @param statement a raptor statement
		 * @param callback the callback to use for handling triples.
		 */
		void add(raptor_statement *statement, const semweb::TripleHandler &callback);

		/**
		 * Flush the parser, pushing all remaining triples to the callback.
		 */
		void flush();

		/**
		 * @return the list of directly imported ontologies.
		 */
		auto &imports() const { return imports_; }

	protected:
		raptor_uri *uri_;
		raptor_uri *uriBase_;
		raptor_parser *parser_;
		raptor_world *world_;
		GraphSelectorPtr frame_;
		semweb::TripleFilter filter_;
		std::string blankPrefix_;
		std::string origin_;
		std::vector<std::string> imports_;
		std::function<int()> doParse_;

		raptor_parser *createParser(knowrob::semweb::TripleFormat format);

		static raptor_world *createWorld();

		static RDFType getLiteralTypeFromURI(const char *typeURI);

		void applyFrame(StatementData *triple);

		class Batch : public semweb::TripleContainer {
		public:
			Batch(semweb::TripleHandler callback, uint32_t size, std::string_view origin);

			~Batch();

			semweb::TripleHandler callback;

			StatementData *add(raptor_statement *statement);

			void rollbackLast();

			void shrink();

			auto size() const { return actualSize_; }

			const std::vector<StatementData> &asVector() const override { return mappedData_; }

		protected:
			std::vector<raptor_statement *> raptorData_;
			std::vector<StatementData> mappedData_;
			uint32_t actualSize_;
			std::string_view origin_;
		};

		std::shared_ptr<Batch> currentBatch_;
		uint32_t batchSize_;
	};

} // knowrob

#endif //KNOWROB_ONTOLOGY_PARSER_H
