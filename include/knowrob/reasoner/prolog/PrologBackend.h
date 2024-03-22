/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_BACKEND_H
#define KNOWROB_PROLOG_BACKEND_H

#include "knowrob/backend/QueryableBackend.h"
#include "PrologTerm.h"

namespace knowrob {
	/**
	 * The triple store backend for Prolog reasoners.
	 * The backend is a singleton, and it is not possible to have multiple instances of it.
	 * The reason is that Prolog reasoners do not support multiple EDBs due to limitations of the
	 * underlying Prolog "semweb" library which only has a global storage.
	 */
	class PrologBackend : public QueryableBackend {
	public:
		PrologBackend();

		bool initializeBackend();

		// override DataBackend
		bool initializeBackend(const PropertyTree &cfg) override;

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

		// override QueryableBackend
		bool isPersistent() const override;

		// override QueryableBackend
		void batch(const TripleHandler &callback) const override;

		// override QueryableBackend
		void query(const GraphQueryPtr &query, const BindingsHandler &callback) override;

		// override QueryableBackend
		void count(const ResourceCounter &callback) const override;

	protected:
		static PrologTerm transaction(std::string_view rdf_functor, const TripleContainerPtr &triples);
	};

} // knowrob

#endif //KNOWROB_PROLOG_BACKEND_H
