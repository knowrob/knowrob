/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/BackendTransaction.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/ReificationContainer.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/reification/ReifiedQuery.h"
#include "knowrob/knowrob.h"

using namespace knowrob;
using namespace knowrob::transaction;

static void setReificationVariable( // NOLINT(misc-no-recursion)
		const std::shared_ptr<GraphTerm> &t,
		const VariablePtr &variable) {
	switch (t->termType()) {
		case knowrob::GraphTermType::Pattern: {
			auto &pattern = std::static_pointer_cast<GraphPattern>(t)->value();
			pattern->setSubjectTerm(variable);
			break;
		}
		case knowrob::GraphTermType::Union:
		case knowrob::GraphTermType::Sequence: {
			auto connective = std::static_pointer_cast<GraphConnective>(t);
			for (auto &term: connective->terms()) {
				setReificationVariable(term, variable);
			}
			break;
		}
		case knowrob::GraphTermType::Builtin:
			break;
	};
}

IRIAtomPtr Transaction::queryReifiedName(const FramedTriple &triple) {
	static auto v_reification = std::make_shared<Variable>("reification");
	auto pat = std::make_shared<FramedTriplePattern>(triple);
	auto query = std::make_shared<GraphPathQuery>(pat);
	auto reified = std::make_shared<ReifiedQuery>(query, vocabulary_);
	setReificationVariable(reified->term(), v_reification);

	IRIAtomPtr reifiedName;
	queryable_->query(reified, [&](const BindingsPtr &bindings) {
		auto t_reifiedName = bindings->get(v_reification->name());
		if (t_reifiedName && t_reifiedName->isIRI()) {
			reifiedName = IRIAtom::Tabled(std::static_pointer_cast<IRIAtom>(t_reifiedName)->stringForm());
		}
	});
	return reifiedName;
}

bool Transaction::commit(const FramedTriple &triple) {
	static auto v_reification = std::make_shared<Variable>("reification");
	if (isRemoval_ && ReifiedTriple::isReifiable(triple)) {
		return commit(triple, queryReifiedName(triple));
	} else {
		return commit(triple, nullptr);
	}
}

bool Transaction::commit(const FramedTriple &triple, const IRIAtomPtr &reifiedName) {
	ReifiedTriplePtr reification;
	bool success = true;
	for (auto &definedBackend: backends_) {
		auto &backend = definedBackend->value();
		if (!backend->supports(BackendFeature::TripleContext) && ReifiedTriple::isReifiable(triple)) {
			if (!reification) reification = std::make_shared<ReifiedTriple>(triple, vocabulary_, reifiedName);
			for (auto &reified: *reification) {
				success = success && doCommit(*reified.ptr, backend);
			}
		} else {
			success = doCommit(triple, backend);
		}
		if (!success) break;
	}
	if (success) {
		updateVocabulary(triple);
	}
	return success;
}

bool Transaction::commit(const TripleContainerPtr &triples) {
	static auto v_reification = std::make_shared<Variable>("reification");
	if (isRemoval_ && !queryable_->supports(BackendFeature::TripleContext)) {
		// FIXME: The name lookup only works in case the queryable backend does store reified triples.
		//        (1) we could search for a queryable backend without context support and use this instead, or
		//        (2) we could store the reified names also in backends that support context, such they
		//            can be queried from any queryable backend.
		// Note: the container type does not provide a size method because it internally uses a generator
		// without knowing when it will end. Also, container with additional filtering could be implemented.
		// So we need to resize the reifiedNames vector while looping over the triples, but we can use
		// the default batch size as initial size.
		auto estimatedSize = GlobalSettings::batchSize();
		ReifiedNames reifiedNames = std::make_shared<std::vector<IRIAtomPtr>>();
		reifiedNames->reserve(estimatedSize);
		for (auto &triple: *triples) {
			if (ReifiedTriple::isReifiable(*triple)) {
				reifiedNames->push_back(queryReifiedName(*triple));
			} else {
				reifiedNames->push_back(nullptr);
			}
		}
		// If fewer elements were added, resize the vector
    	if (reifiedNames->size() < estimatedSize) {
        	reifiedNames->resize(reifiedNames->size());
    	}
		return commit(triples, reifiedNames);
	} else {
		return commit(triples, {});
	}
}

bool Transaction::commit(const TripleContainerPtr &triples, const ReifiedNames &reifiedNames) {
	TripleContainerPtr reified;
	std::vector<std::shared_ptr<ThreadPool::Runner>> transactions;
	bool success = true;

	auto vocabWorker = createTripleWorker(triples,
										  [this](const FramedTriplePtr &triple) { updateVocabulary(*triple); });

	for (auto &definedBackend: backends_) {
		auto &backend = definedBackend->value();
		const TripleContainerPtr *backendTriples;
		if (!backend->supports(BackendFeature::TripleContext)) {
			if (!reified) reified = std::make_shared<ReificationContainer>(triples, vocabulary_, reifiedNames);
			backendTriples = &reified;
		} else {
			backendTriples = &triples;
		}
		auto worker = std::make_shared<ThreadPool::LambdaRunner>(
				[&](const std::function<bool()> &) { success = success && doCommit(*backendTriples, backend); });
		transactions.push_back(worker);

		DefaultThreadPool()->pushWork(worker,
									  [&definedBackend](const std::exception &exc) {
										  KB_ERROR("transaction failed for backend '{}': {}", definedBackend->name(),
												   exc.what());
									  });
	}

	for (auto &transaction: transactions) transaction->join();
	vocabWorker->join();

	return success;
}

std::shared_ptr<ThreadPool::Runner> Transaction::createTripleWorker(
		const TripleContainerPtr &triples,
		const std::function<void(const FramedTriplePtr &)> &fn) {
	auto perTripleWorker =
			std::make_shared<ThreadPool::LambdaRunner>([fn, triples](const ThreadPool::LambdaRunner::StopChecker &) {
				std::for_each(triples->begin(), triples->end(), fn);
			});
	DefaultThreadPool()->pushWork(perTripleWorker,
								  [](const std::exception &exc) {
									  KB_ERROR("failed to update vocabulary: {}", exc.what());
								  });
	return perTripleWorker;
}

bool Insert::doCommit(const FramedTriple &triple, const DataBackendPtr &backend) {
	return backend->insertOne(triple);
}

bool Remove::doCommit(const FramedTriple &triple, const DataBackendPtr &backend) {
	return backend->removeOne(triple);
}

bool Insert::doCommit(const TripleContainerPtr &triples, const knowrob::DataBackendPtr &backend) {
	return backend->insertAll(triples);
}

bool Remove::doCommit(const TripleContainerPtr &triples, const knowrob::DataBackendPtr &backend) {
	return backend->removeAll(triples);
}

void Insert::updateVocabulary(const FramedTriple &triple) {
	// keep track of imports, subclasses, and subproperties
	if (isSubClassOfIRI(triple.predicate())) {
		auto sub = vocabulary_->defineClass(triple.subject());
		auto sup = vocabulary_->defineClass(triple.valueAsString());
		sub->addDirectParent(sup);
	} else if (isSubPropertyOfIRI(triple.predicate())) {
		auto sub = vocabulary_->defineProperty(triple.subject());
		auto sup = vocabulary_->defineProperty(triple.valueAsString());
		sub->addDirectParent(sup);
	} else if (isTypeIRI(triple.predicate())) {
		vocabulary_->addResourceType(triple.subject(), triple.valueAsString());
		// increase frequency in vocabulary
		static std::set<std::string_view> skippedTypes = {
				owl::Class->stringForm(),
				owl::Restriction->stringForm(),
				owl::NamedIndividual->stringForm(),
				owl::AnnotationProperty->stringForm(),
				owl::ObjectProperty->stringForm(),
				owl::DatatypeProperty->stringForm(),
				rdfs::Class->stringForm(),
				rdf::Property->stringForm()
		};
		if (vocabulary_->isDefinedClass(triple.valueAsString()) &&
			!skippedTypes.count(triple.valueAsString())) {
			vocabulary_->increaseFrequency(triple.valueAsString());
		}
	} else if (isInverseOfIRI(triple.predicate())) {
		auto p = vocabulary_->defineProperty(triple.subject());
		auto q = vocabulary_->defineProperty(triple.valueAsString());
		p->setInverse(q);
		q->setInverse(p);
	} else if (owl::imports->stringForm() == triple.predicate()) {
		auto resolvedImport = URI::resolve(triple.valueAsString());
		auto importedGraph = DataSource::getNameFromURI(resolvedImport);
		if (triple.graph()) {
			vocabulary_->importHierarchy()->addDirectImport(triple.graph().value(), importedGraph);
		} else {
			KB_WARN("import statement without graph");
		}
	} else if (vocabulary_->isObjectProperty(triple.predicate()) ||
			   vocabulary_->isDatatypeProperty(triple.predicate())) {
		// increase frequency of property in vocabulary
		vocabulary_->increaseFrequency(triple.predicate());
	}
}

void Remove::updateVocabulary(const FramedTriple &triple) {
	// TODO: a triple can have multiple origins, so updating the vocabulary on removal is not safe without
	//  knowing if the triple has other origins. But (1) there is no interface yet to query for the origins of
	//  a triple, and (2) maybe it can be avoided to do additional querying here.
#if 0
	// remove subclass and subproperty relations from the vocabulary.
	if (isSubClassOfIRI(triple.predicate())) {
		auto sub = vocabulary_->defineClass(triple.subject());
		auto sup = vocabulary_->defineClass(triple.valueAsString());
		sub->removeDirectParent(sup);
	} else if (isSubPropertyOfIRI(triple.predicate())) {
		auto sub = vocabulary_->defineProperty(triple.subject());
		auto sup = vocabulary_->defineProperty(triple.valueAsString());
		sub->removeDirectParent(sup);
	}
#endif
}
