/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PERSISTENT_BACKEND_H
#define KNOWROB_PERSISTENT_BACKEND_H

#include "optional"
#include "string"
#include "memory"

namespace knowrob {
	/**
	 * Interface for a persistent backend.
	 */
	class PersistentBackend {
	public:
		/**
		 * @param origin an origin string.
		 * @return the version of the origin, or an empty optional if the origin is unknown.
		 */
		virtual std::optional<std::string> getVersionOfOrigin(std::string_view origin) = 0;

		/**
		 * Set the version of an origin.
		 * @param origin an origin string.
		 * @param version a version string.
		 */
		virtual void setVersionOfOrigin(std::string_view origin, std::string_view version) = 0;
	};

	using PersistentBackendPtr = std::shared_ptr<PersistentBackend>;
}

#endif //KNOWROB_PERSISTENT_BACKEND_H
