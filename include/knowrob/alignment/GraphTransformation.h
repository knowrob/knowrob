/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_TRANSFORMATION_H
#define KNOWROB_GRAPH_TRANSFORMATION_H

#include <functional>
#include <boost/property_tree/ptree.hpp>
#include "knowrob/triples/TripleContainer.h"

namespace knowrob {
	/**
	 * Base class for graph transformations.
	 * Each transformation consumes input triples and produces output triples.
	 * The output triples are either passed to the next transformation or to a handler.
	 * Note that GraphTransformation currently maybe not preserve the context of input triples,
	 * and that the context of output triples is limited to reflect the triple origin.
	 */
	class GraphTransformation {
	public:
		/**
		 * Set the next transformation to be called after this one.
		 * @param next the next transformation
		 */
		void setNext(const std::shared_ptr<GraphTransformation> &next) { nextTransformation_ = next; }

		/**
		 * Set the next handler to be called after this transformation.
		 * @param next the next handler
		 */
		void setNext(const TripleHandler &next) { next_ = next; }

		/**
		 * Set the origin of the triples.
		 * Transformations maybe do not preserve context of input triples, so the origin of the input triples
		 * must be specified.
		 * @param origin the origin of the input triples
		 */
		void setOrigin(std::string_view origin) { origin_ = origin; }

		/**
		 * Get the origin of the triples.
		 * @return the origin of the input triples
		 */
		auto origin() const { return origin_; }

		/**
		 * Push output triples to the next transformation or handler.
		 * @param triples the output triples
		 */
		void pushOutputTriples(const TripleContainerPtr &triples);

		/**
		 * Push input triples to the transformation.
		 * @param triples the input triples
		 */
		virtual void pushInputTriples(const TripleContainerPtr &triples) = 0;

		/**
		 * Configure the transformation with the given options.
		 * @param opts the options
		 * @return true if the configuration was successful
		 */
		virtual bool configure(const boost::property_tree::ptree &opts) = 0;

		/**
		 * Initialize the transformation.
		 */
		virtual void initializeTransformation() = 0;

		/**
		 * Finalize the transformation.
		 */
		virtual void finalizeTransformation() = 0;

	protected:
		TripleHandler next_;
		std::shared_ptr<GraphTransformation> nextTransformation_;
		std::string origin_;

		void initializeNext();

		void finalizeNext();
	};

} // knowrob

#endif //KNOWROB_GRAPH_TRANSFORMATION_H
