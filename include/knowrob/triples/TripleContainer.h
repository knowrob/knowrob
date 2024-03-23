/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TRIPLE_CONTAINER_H
#define KNOWROB_TRIPLE_CONTAINER_H

#include <boost/iterator/iterator_facade.hpp>
#include <utility>
#include <vector>
#include <memory>
#include <functional>
#include "knowrob/triples/FramedTriple.h"

namespace knowrob {
	/**
	 * A container of framed triples that can be created from a generator function.
	 * The baseclass has immutable triples, but `MutableTripleContainer` is a subclass that allows to modify
	 * triples in the container.
	 */
	class TripleContainer {
	public:
		/**
		 * A generator function that returns a const pointer to a triple.
		 */
		using ConstGenerator = std::function<const FramedTriplePtr *()>;

		/**
		 * An iterator that can be used to iterate over the triples in the container by
		 * using a const generator function.
		 */
		struct iterator : boost::iterator_facade<iterator, const FramedTriplePtr, boost::forward_traversal_tag> {
			explicit iterator(const ConstGenerator &generator)
					: generator_(generator), ptr_(generator_()) {
			}

			bool equal(iterator const &other) const { return ptr_ == other.ptr_; }

			const FramedTriplePtr &dereference() const { return *ptr_; }

			void increment() { ptr_ = generator_(); }

		private:
			ConstGenerator generator_;
			const FramedTriplePtr *ptr_;
		};

		/**
		 * @return an iterator that points to the first const triple in the container.
		 */
		iterator begin() const {
			return iterator(cgenerator());
		}

		/**
		 * @return an iterator that points to the end of the container.
		 */
		static iterator end() {
			return iterator([] { return nullptr; });
		}

		/**
		 * @return true if the container is empty.
		 */
		bool empty() const {
			return begin().equal(end());
		}

		/**
		 * @return true if the container is mutable.
		 */
		virtual bool isMutable() const { return false; }

		/**
		 * @return a generator function that returns a const pointer to a triple.
		 */
		virtual ConstGenerator cgenerator() const = 0;
	};

	/**
	 * A triple container with mutable triples.
	 */
	class MutableTripleContainer : public TripleContainer {
	public:
		/**
		 * A generator function that returns a pointer to a triple.
		 */
		using MutableGenerator = std::function<FramedTriplePtr *()>;

		/**
		 * An iterator that can be used to iterate over the triples in the container by
		 * using a generator function.
		 */
		struct iterator : boost::iterator_facade<iterator, FramedTriplePtr, boost::forward_traversal_tag> {
			explicit iterator(MutableGenerator generator)
					: generator_(std::move(generator)), ptr_(generator_()) {
			}

			bool equal(iterator const &other) const { return ptr_ == other.ptr_; }

			FramedTriplePtr &dereference() const { return *ptr_; }

			void increment() { ptr_ = generator_(); }

		private:
			MutableGenerator generator_;
			FramedTriplePtr *ptr_;
		};

		/**
		 * @return an iterator that points to the first triple in the container.
		 */
		iterator begin() {
			return iterator(generator());
		}

		/**
		 * @return an iterator that points to the end of the container.
		 */
		static iterator end() {
			return iterator([] { return nullptr; });
		}

		/**
		 * @return a generator function that returns a pointer to a triple.
		 */
		virtual MutableGenerator generator() = 0;

		// Override TripleContainer
		bool isMutable() const override { return true; }
	};

	/**
	 * A triple container that loops over a pointer of a vector of framed triples.
	 */
	class ProxyTripleContainer : public TripleContainer {
	public:
		explicit ProxyTripleContainer(const std::vector<FramedTriplePtr> *triples);

		// Override TripleContainer
		ConstGenerator cgenerator() const override;

	protected:
		const std::vector<FramedTriplePtr> *triples_;
	};

	/**
	 * A batch of framed triples that attempts to take over ownership
	 * of the triples that are added to it.
	 */
	class TripleViewBatch : public MutableTripleContainer {
	public:
		explicit TripleViewBatch(uint32_t batchSize);

		/**
		 * @return the actual size of the batch.
		 */
		auto size() const { return actualSize_; }

		/**
		 * Resets the batch to an empty state.
		 */
		void reset() { actualSize_ = 0; }

		/**
		 * Adds a triple to the batch, and attempts to take over ownership of the triple.
		 * If this fails, a new copy of the triple is created and added to the batch.
		 * @param triple a triple.
		 */
		void add(const FramedTriplePtr &triple);

		// Override TripleContainer
		ConstGenerator cgenerator() const override;

		// Override MutableTripleContainer
		MutableGenerator generator() override;

	protected:
		std::vector<FramedTriplePtr> data_;
		uint32_t batchSize_;
		uint32_t actualSize_;
	};

	using TripleContainerPtr = std::shared_ptr<TripleContainer>;
	using TripleHandler = std::function<void(const TripleContainerPtr &)>;
	using TripleVisitor = std::function<void(const FramedTriplePtr &)>;

	using MutableTripleContainerPtr = std::shared_ptr<MutableTripleContainer>;
	using MutableTripleHandler = std::function<void(const MutableTripleContainerPtr &)>;

	using TripleFilter = std::function<bool(const FramedTriple &)>;
}

#endif //KNOWROB_TRIPLE_CONTAINER_H
