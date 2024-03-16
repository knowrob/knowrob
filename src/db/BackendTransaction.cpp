/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/BackendTransaction.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/ReificationContainer.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/rdf.h"

using namespace knowrob;
using namespace knowrob::transaction;

bool Transaction::commit(const FramedTriple &triple) {
	ReifiedTriplePtr reification;
	bool success = true;
	for (auto &definedBackend: backends_) {
		auto &backend = definedBackend->backend();
		if (!backend->supports(BackendFeature::TripleContext) && ReifiedTriple::isReifiable(triple)) {
			if (!reification) reification = std::make_shared<ReifiedTriple>(triple, vocabulary_);
			for (auto &reified: *reification) {
				success = success && commit(*reified.ptr, backend);
			}
		} else {
			success = commit(triple, backend);
		}
		if (!success) break;
	}
	if (success) {
		updateVocabulary(triple);
	}
	return success;
}

bool Transaction::commit(const semweb::TripleContainerPtr &triples) {
	semweb::TripleContainerPtr reified;
	std::vector<std::shared_ptr<ThreadPool::Runner>> transactions;
	bool success = true;

	auto vocabWorker = createTripleWorker(triples,
										  [this](const FramedTriplePtr &triple) { updateVocabulary(*triple); });

	for (auto &definedBackend: backends_) {
		auto &backend = definedBackend->backend();
		const semweb::TripleContainerPtr *backendTriples;
		if (!backend->supports(BackendFeature::TripleContext)) {
			// FIXME: there is a problem with reification and removal, individual name CANNOT be generated.
			//        so need to allow that an additional mapping from triple to reified individual is provided.
			if (!reified) reified = std::make_shared<ReificationContainer>(triples, vocabulary_);
			backendTriples = &reified;
		} else {
			backendTriples = &triples;
		}
		auto worker = std::make_shared<ThreadPool::LambdaRunner>(
				[&](const std::function<bool()> &) { success = success && commit(*backendTriples, backend); });
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
		const semweb::TripleContainerPtr &triples,
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

bool Insert::commit(const FramedTriple &triple, const DataBackendPtr &backend) {
	return backend->insertOne(triple);
}

bool Remove::commit(const FramedTriple &triple, const DataBackendPtr &backend) {
	return backend->removeOne(triple);
}

bool Insert::commit(const semweb::TripleContainerPtr &triples, const knowrob::DataBackendPtr &backend) {
	return backend->insertAll(triples);
}

bool Remove::commit(const semweb::TripleContainerPtr &triples, const knowrob::DataBackendPtr &backend) {
	return backend->removeAll(triples);
}

void Insert::updateVocabulary(const FramedTriple &triple) {
	// keep track of imports, subclasses, and subproperties
	if (semweb::isSubClassOfIRI(triple.predicate())) {
		auto sub = vocabulary_->defineClass(triple.subject());
		auto sup = vocabulary_->defineClass(triple.valueAsString());
		sub->addDirectParent(sup);
	} else if (semweb::isSubPropertyOfIRI(triple.predicate())) {
		auto sub = vocabulary_->defineProperty(triple.subject());
		auto sup = vocabulary_->defineProperty(triple.valueAsString());
		sub->addDirectParent(sup);
	} else if (semweb::isTypeIRI(triple.predicate())) {
		vocabulary_->addResourceType(triple.subject(), triple.valueAsString());
		// increase frequency in vocabulary
		static std::set<std::string_view> skippedTypes = {
				semweb::owl::Class->stringForm(),
				semweb::owl::Restriction->stringForm(),
				semweb::owl::NamedIndividual->stringForm(),
				semweb::owl::AnnotationProperty->stringForm(),
				semweb::owl::ObjectProperty->stringForm(),
				semweb::owl::DatatypeProperty->stringForm(),
				semweb::rdfs::Class->stringForm(),
				semweb::rdf::Property->stringForm()
		};
		if (vocabulary_->isDefinedClass(triple.valueAsString()) &&
			!skippedTypes.count(triple.valueAsString())) {
			vocabulary_->increaseFrequency(triple.valueAsString());
		}
	} else if (semweb::isInverseOfIRI(triple.predicate())) {
		auto p = vocabulary_->defineProperty(triple.subject());
		auto q = vocabulary_->defineProperty(triple.valueAsString());
		p->setInverse(q);
		q->setInverse(p);
	} else if (semweb::owl::imports->stringForm() == triple.predicate()) {
		auto resolvedImport = URI::resolve(triple.valueAsString());
		auto importedGraph = DataSource::getNameFromURI(resolvedImport);
		if (triple.graph()) {
			importHierarchy_->addDirectImport(triple.graph().value(), importedGraph);
		} else {
			KB_WARN("import statement without graph");
		}
	} else if (vocabulary_->isObjectProperty(triple.predicate()) ||
			   vocabulary_->isDatatypeProperty(triple.predicate())) {
		// increase frequency of property in vocabulary
		vocabulary_->increaseFrequency(triple.predicate());
	}
}

void Remove::updateVocabulary(const FramedTriple &tripleData) {
	// TODO: implement
}
