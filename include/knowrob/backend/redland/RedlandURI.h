/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REDLAND_URI_H
#define KNOWROB_REDLAND_URI_H

#include <redland.h>
#include <string_view>

namespace knowrob {
	/**
	 * A wrapper for librdf_uri.
	 * Note that the URI is a null pointer after construction and must be set with set().
	 */
	class RedlandURI {
	public:
		RedlandURI() : uri_(nullptr) {}

		~RedlandURI() { if (uri_) librdf_free_uri(uri_); }

		RedlandURI(const RedlandURI &) = delete;

		/**
		 * @return the librdf uri
		 */
		auto operator()() const { return uri_; }

		/**
		 * @param world a librdf world
		 * @param uri a URI string
		 */
		void set(librdf_world *world, std::string_view uri) {
			if (uri_) librdf_free_uri(uri_);
			uri_ = librdf_new_uri(world, (const unsigned char *) uri.data());
		}

	protected:
		librdf_uri *uri_;
	};
}

#endif //KNOWROB_REDLAND_URI_H
