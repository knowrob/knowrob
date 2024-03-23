/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_EVENT_H
#define KNOWROB_REASONER_EVENT_H

#include <vector>
#include <memory>
#include <string>
#include "knowrob/triples/FramedTriple.h"

namespace knowrob::reasoner {
	/**
	 * An event that is emitted by a reasoner.
	 */
	class Event {
	public:
		/**
		 * The type of the event.
		 */
		enum Type {
			/**
			 * The reasoner has inferred a new statement.
			 */
			Assertion,
			/**
			 * The reasoner has retracted a statement.
			 */
			Retraction,
			/**
			 * The reasoner signals that all previous statements should be replaced by new ones.
			 */
			Replacement,
			/**
			 * The reasoner signals that its update method needs to be called.
			 */
			Invalidation
		};

		explicit Event(Type eventType) : eventType_(eventType) {}

		/**
		 * @return the event type.
		 */
		Type eventType() const { return eventType_; }

	protected:
		const Type eventType_;
	};

	/**
	 * An event that contains a set of triples.
	 */
	class TripleEvent : public Event {
	public:
		/**
		 * @param eventType the type of the event.
		 * @param tripleCount the number of triples in this event.
		 * @param copy whether to use std::string (true) or std::string_view (false) for triples.
		 */
		TripleEvent(Type eventType, uint32_t tripleCount, bool copy = true);

		/**
		 * @return the triples of this event.
		 */
		auto &triples() const { return triples_; }

		/**
		 * @return the triple at the given index.
		 */
		auto &triple(uint32_t index) { return *triples_[index].ptr; }

	protected:
		std::vector<FramedTriplePtr> triples_;
	};

	/**
	 * An event that contains a set of assertions.
	 * These will be added to the set of inferred triples of this reasoner,
	 * and added to the knowledge base.
	 */
	class AssertionEvent : public TripleEvent {
	public:
		/**
		 * @param numTriples the number of triples in this event.
		 * @param copy whether to use std::string (true) or std::string_view (false) for triples.
		 */
		explicit AssertionEvent(uint32_t numTriples, bool copy = true)
				: TripleEvent(Assertion, numTriples, copy) {}
	};

	/**
	 * An event that contains a set of retractions.
	 * These will be removed from the set of inferred triples of this reasoner,
	 * and removed from the knowledge base.
	 */
	class RetractionEvent : public TripleEvent {
	public:
		/**
		 * @param numTriples the number of triples in this event.
		 * @param copy whether to use std::string (true) or std::string_view (false) for triples.
		 */
		explicit RetractionEvent(uint32_t numTriples, bool copy = true)
				: TripleEvent(Retraction, numTriples, copy) {}
	};

	/**
	 * An event that contains a set of replacements.
	 * These will replace the triples that were previously asserted.
	 * This is supposed to be used by reasoner that do not keep track of
	 * incremental changes.
	 * In case of incremental reasoners, the assertion and retraction events
	 * should rather be used.
	 */
	class ReplacementEvent : public TripleEvent {
	public:
		/**
		 * @param numTriples the number of triples in this event.
		 * @param copy whether to use std::string (true) or std::string_view (false) for triples.
		 */
		explicit ReplacementEvent(uint32_t numTriples, bool copy = true)
				: TripleEvent(Replacement, numTriples, copy) {}
	};

	/**
	 * An event that signals that the reasoner needs to be updated.
	 * Sending such events is considered an optional capability that needs to
	 * be an enabled feature of the reasoner.
	 */
	class InvalidationEvent : public Event {
	public:
		InvalidationEvent() : Event(Invalidation) {}
	};

} // knowrob

#endif //KNOWROB_REASONER_EVENT_H
