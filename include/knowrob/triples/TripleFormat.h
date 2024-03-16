/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TRIPLE_FORMAT_H
#define KNOWROB_TRIPLE_FORMAT_H

#include "string_view"

namespace knowrob::semweb {
	/**
	 * Used to indicate the file format when loading triple data.
	 */
	enum TripleFormat {
		RDF_XML,
		RDFA,
		TRIG,
		GRDDL,
		TURTLE,
		N_TRIPLES
	};

	TripleFormat tripleFormatFromString(std::string_view format);

	std::string_view tripleFormatToString(TripleFormat format);

	std::string_view tripleFormatMimeType(TripleFormat format);

	bool isTripleFormatString(std::string_view format);
}

#endif //KNOWROB_TRIPLE_FORMAT_H
