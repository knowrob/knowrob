/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_H_
#define KNOWROB_REASONER_H_

#include <memory>
#include <filesystem>
#include <fmt/core.h>

#include "knowrob/terms/Term.h"
#include "knowrob/PropertyTree.h"
#include "knowrob/sources//DataSource.h"
#include "knowrob/queries/TokenBuffer.h"
#include "knowrob/formulas/FirstOrderLiteral.h"
#include "knowrob/formulas/PredicateDescription.h"
#include "knowrob/triples/GraphQuery.h"
#include "knowrob/sources/DataSourceHandler.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/backend/Backend.h"
#include "knowrob/terms/Atom.h"

namespace knowrob {
	// forward declarations
	class KnowledgeBase;

	class ReasonerManager;

	using InferredTripleContainer = std::shared_ptr<std::vector<std::shared_ptr<FramedTriple>>>;

	/**
	 * A reasoner is a component that can infer new knowledge.
	 * It does so by evaluating axioms or rules that are defined by the reasoner.
	 * The axioms and rules may refer to extensional data which is stored in the
	 * knowledge base in form of a knowledge graph, and which may need to be mirrored
	 * into the reasoner's own data backend in case it cannot operate directly
	 * on the central extensional database.
	 * Note that a reasoner is also a data source handler, i.e. data which is needed
	 * by the reasoner to operate which is not stored in a backend.
	 */
	class Reasoner : public DataSourceHandler {
	public:
		Reasoner();

		virtual ~Reasoner() = default;

		/**
		 * @return name of the reasoner name.
		 */
		auto reasonerName() const { return t_reasonerName_->stringForm(); }

		/**
		 * @return a term representing the reasoner name.
		 */
		auto &reasonerNameTerm() const { return t_reasonerName_; }

		/**
		 * @return the reasoner manager associated with this reasoner.
		 */
		ReasonerManager &reasonerManager() const;

		/**
		 * Note that this will raise an exception if the reasoner is not associated with a knowledge base.
		 * @return the knowledge base that this reasoner is associated with.
		 */
		KnowledgeBase *kb() const;

		/**
		 * @return the vocabulary of this backend.
		 */
		std::shared_ptr<Vocabulary> vocabulary() const;

		/**
		 * Evaluate a lambda function in a worker thread.
		 * @param fn a function to be executed.
		 */
		void pushWork(const std::function<void(void)> &fn);

		/**
		 * Set the data backend of this reasoner.
		 */
		virtual void setDataBackend(const DataBackendPtr &backend) = 0;

		/**
		 * Load a reasoner configuration.
		 * The knowledge base system only calls this function once for each reasoner instance.
		 * @param config a ReasonerConfig object.
		 */
		virtual bool loadConfig(const PropertyTree &config) = 0;

		/**
		 * Get the description of a predicate currently defined by this reasoner.
		 * A predicate is thought to be currently defined if the reasoner can submitQuery it.
		 *
		 * @param indicator a predicate indicator
		 * @return a predicate description if the predicate is a defined one or null otherwise.
		 */
		virtual PredicateDescriptionPtr getDescription(const PredicateIndicatorPtr &indicator) = 0;

		/**
		 * Get the description of the predicate which is associated with a literal.
		 * A null reference will be returned in case that the property term of the literal is a variable.
		 * @param literal a literal.
		 * @return a predicate description or a null reference.
		 */
		PredicateDescriptionPtr getLiteralDescription(const FramedTriplePattern &literal);

		/**
		 * Start the reasoner.
		 * This is in particular intended to start any bottom-up evaluation processes
		 * that run in the background.
		 */
		virtual void start() = 0;

		/**
		 * Stop the reasoner, and destroy all resources.
		 * No further queries can be submitted after this function has been called.
		 */
		virtual void stop() = 0;

		/**
		 * Submit a query to the reasoner.
		 * The query is represented by a literal and a context.
		 * The evaluation of the query is performed asynchronously, the result of this function
		 * is a buffer that can be used to retrieve the results of the query at a later point in time.
		 * @param literal a literal representing the query.
		 * @param ctx a query context.
		 * @return a buffer that can be used to retrieve the results of the query.
		 */
		virtual TokenBufferPtr submitQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx) = 0;

		/**
		 * Create a vector of triples that can be used to insert or remove data from the reasoner's data backend.
		 * @param count the number of triples to create.
		 * @return a vector of triples.
		 */
		InferredTripleContainer createTriples(uint32_t count) const;

		/**
		 * Set the inferred triples of this reasoner.
		 * Calling this function will replace the current set of inferred triples, and will
		 * delete all triples from the KB that are not in the inferred set anymore, and add all triples
		 * to the KB that newly appear.
		 * @param triples a vector of inferred triples.
		 */
		void setInferredTriples(const InferredTripleContainer &triples);

		/**
		 * Add additional triples to the inferred set of this reasoner,
		 * and adding the inferred triples to the KB.
		 * @param triples a vector of inferred triples.
		 */
		void addInferredTriples(const InferredTripleContainer &triples) const;

		/**
		 * Remove triples from the inferred set of this reasoner,
		 * and remove the inferred triples from the KB.
		 * @param triples a vector of inferred triples.
		 */
		void removeInferredTriples(const InferredTripleContainer &triples) const;

	private:
		AtomPtr t_reasonerName_;

		struct InferredComparator {
			bool operator()(const std::shared_ptr<FramedTriple> &v0,
							const std::shared_ptr<FramedTriple> &v1) const;
		};

		using InferredeSet = std::set<std::shared_ptr<FramedTriple>, InferredComparator>;
		InferredeSet inferredTriples_;

		friend class ReasonerManager;

		ReasonerManager *reasonerManager_;

		void setReasonerManager(ReasonerManager *reasonerManager);

		void setReasonerName(std::string_view name);
	};

	/**
	 * In some cases reasoner and backend cannot really be separated.
	 * The whole point of the separation is to re-use backends with different reasoners.
	 * So if this is not possible, then the reasoner can also implement the backend interface.
	 */
	class ReasonerWithBackend : public Reasoner, public DataBackend {
	public:
		ReasonerWithBackend() : Reasoner(), DataBackend() {}

		// avoid a self-reference
		void setDataBackend(const DataBackendPtr &backend) final {}
	};
}

#endif //KNOWROB_REASONER_H_
