/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_PARSER_H
#define KNOWROB_ONTOLOGY_PARSER_H

#include <functional>
#include <string>
#include <raptor.h>
#include "knowrob/triples/TripleContainer.h"
#include "knowrob/triples/TripleFormat.h"
#include "knowrob/triples/FramedTriple.h"
#include "knowrob/triples/GraphSelector.h"
#include "RaptorContainer.h"

namespace knowrob {
	/**
	 * A parser for RDF data.
	 */
	class OntologyParser {
	public:
		/**
		 * @param fileURI the URI of the file to parse.
		 * @param format the format of the file.
		 * @param batchSize the max size of each batch to use for parsing.
		 */
		OntologyParser(const std::string_view &fileURI, semweb::TripleFormat format);

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
		void setFilter(const TripleFilter &filter) { filter_ = filter; }

		/**
		 * @param callback the callback to use for handling triples.
		 * @return true if the parsing was successful, false otherwise.
		 */
		bool run(const TripleHandler &callback);

		/**
		 * @param statement a raptor statement
		 * @param callback the callback to use for handling triples.
		 */
		void add(raptor_statement *statement, const TripleHandler &callback);

		/**
		 * Flush the parser, pushing all remaining triples to the callback.
		 */
		void flush(const TripleHandler &callback);

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
		TripleFilter filter_;
		std::string blankPrefix_;
		std::string origin_;
		std::vector<std::string> imports_;
		std::function<int()> doParse_;

		raptor_parser *createParser(knowrob::semweb::TripleFormat format);

		static raptor_world *createWorld();

		void applyFrame(FramedTriple *triple);

		std::shared_ptr<RaptorContainer> currentBatch_;
	};

} // knowrob

#endif //KNOWROB_ONTOLOGY_PARSER_H
