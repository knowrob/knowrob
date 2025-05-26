/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TRIPLE_FORMATTER_H
#define KNOWROB_TRIPLE_FORMATTER_H

#include "string_view"
#include "map"
#include "vector"
#include "knowrob/semweb/TripleFormat.h"
#include "knowrob/semweb/Triple.h"

namespace knowrob::semweb {
	using ExportedTriples = std::map<std::string_view, std::vector<std::shared_ptr<Triple>>>;

    /**
     * A class that is responsible for exporting triples to different formats.
     */
    class TripleFormatter {
    public:
        /**
         * @param triples a map of triples
         * @param filename the name of the file to export to
         * @param format the format of the output file
         * @return true if the export was successful
         */
        static bool exportTo(
            const ExportedTriples &triples,
            const std::string &filename,
            TripleFormat format = TripleFormat::RDF_XML);

        /**
         * @param triples a map of triples
         * @param filename the name of the file to export to
         * @return true if the export was successful
         */
        static bool exportRDF_XML(
            const ExportedTriples &triples,
            const std::string &filename);

        /**
         * @param triples a map of triples
         * @param filename the name of the file to export to
         * @return true if the export was successful
         */
        static bool exportTurtle(
            const ExportedTriples &triples,
            const std::string &filename);
    };
}

#endif //KNOWROB_TRIPLE_FORMATTER_H
