/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PERSPECTIVE_H
#define KNOWROB_PERSPECTIVE_H

#include <utility>

#include "memory"
#include "map"
#include "string"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob {
	/**
	 * A perspective is a point of view from which a statement is true or false.
	 * Usually a perspective is associated with an IRI denoting an agent or a group of agents.
	 */
	class Perspective {
	public:
		/**
		 * Create a new perspective with the given IRI.
		 * @param iri the IRI of the perspective
		 */
		explicit Perspective(std::string_view iri);

		/**
		 * Create a new perspective with the given IRI atom.
		 * @param atom the IRI atom of the perspective
		 */
		explicit Perspective(IRIAtomPtr atom) : atom_(std::move(atom)) {}

		/**
		 * @return the IRI of the perspective
		 */
		auto iri() const { return atom_->stringForm(); }

		/**
		 * @return the IRI atom of the perspective
		 */
		auto atom() const { return atom_; }

		/**
		 * The egoIRI perspective is a special perspective taken by the agent running the knowledge base.
		 * @return the egoIRI perspective
		 */
		static std::shared_ptr<Perspective> getEgoPerspective();

		/**
		 * @param iri an IRI
		 * @return true if the IRI denotes the egoIRI perspective
		 */
		static bool isEgoPerspective(std::string_view iri);

		/**
		 * @param iri an IRI
		 * @return the perspective with the given IRI
		 */
		static std::shared_ptr<Perspective> get(std::string_view iri);

	protected:
		static std::map<std::string_view, std::shared_ptr<Perspective>> perspectiveMap_;
		IRIAtomPtr atom_;
	};

    using PerspectivePtr = std::shared_ptr<Perspective>;

} // knowrob

#endif //KNOWROB_PERSPECTIVE_H
