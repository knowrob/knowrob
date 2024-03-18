/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_ERROR_H_
#define KNOWROB_ONTOLOGY_ERROR_H_

#include <fmt/core.h>
#include <knowrob/KnowRobError.h>

namespace knowrob {
	/**
	 * A data backend-related runtime error.
	 */
	class OntologyError : public KnowRobError {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit OntologyError(const char *fmt, Args&& ... args)
			: KnowRobError("OntologyError", fmt::format(fmt, args...)) {}
	};
}

#endif //KNOWROB_ONTOLOGY_ERROR_H_
