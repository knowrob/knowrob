/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/db/QueryableBackend.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/UnReificationContainer.h"
#include "knowrob/reification/ReifiedQuery.h"
#include "knowrob/triples/GraphBuiltin.h"

using namespace knowrob;

AtomPtr QueryableBackend::versionProperty = IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#hasVersionOfOrigin");

QueryableBackend::QueryableBackend() : batchSize_(1000) {
}

std::vector<std::string> QueryableBackend::getOrigins() {
	static auto v_origin = std::make_shared<Variable>("Origin");
	static auto v_version = std::make_shared<Variable>("Version");
	std::vector<std::string> origins;
	match(FramedTriplePattern(v_origin, versionProperty, v_version),
	      [&](const FramedTriple &triple) { origins.emplace_back(triple.subject()); });
	return origins;
}

void QueryableBackend::setVersionOfOrigin(std::string_view origin, std::string_view version) {
	FramedTripleView triple;
	triple.setSubject(origin);
	triple.setPredicate(versionProperty->stringForm());
	triple.setStringValue(version);
	triple.setGraph(origin);
	insertOne(triple);
}

std::optional<std::string> QueryableBackend::getVersionOfOrigin(std::string_view origin) {
	static auto v_version = std::make_shared<Variable>("Version");
	std::optional<std::string> version;
	match(FramedTriplePattern(std::make_shared<Atom>(origin), versionProperty, v_version),
	      [&](const FramedTriple &triple) { version = triple.createStringValue(); });
	return version;
}

void QueryableBackend::evaluateQuery(const GraphPathQueryPtr &q, const TokenBufferPtr &resultStream) {
	auto channel = TokenStream::Channel::create(resultStream);
	try {
		bool hasPositiveAnswer = false;
		query(q, [&](const FramedBindingsPtr &bindings) {
			channel->push(yes(q, bindings));
			hasPositiveAnswer = true;
		});
		if (!hasPositiveAnswer) {
			channel->push(no(q));
		}
		channel->push(EndOfEvaluation::get());
	}
	catch (const std::exception &e) {
		// make sure EOS is pushed to the stream
		channel->push(EndOfEvaluation::get());
		throw;
	}
}

TokenBufferPtr QueryableBackend::submitQuery(const GraphPathQueryPtr &q) {
	std::shared_ptr<TokenBuffer> result = std::make_shared<TokenBuffer>();
	auto runner =
			std::make_shared<ThreadPool::LambdaRunner>(
					[this, q, result](const ThreadPool::LambdaRunner::StopChecker &) {
						evaluateQuery(q, result);
					});
	DefaultThreadPool()->pushWork(runner, [result, q](const std::exception &e) {
		KB_WARN("an exception occurred for graph query ({}): {}.", *q, e.what());
		result->close();
	});
	return result;
}

AnswerPtr QueryableBackend::yes(const GraphPathQueryPtr &q, const FramedBindingsPtr &bindings) {
	static const auto edbTerm = Atom::Tabled("EDB");
	auto positiveAnswer = std::make_shared<AnswerYes>(bindings);
	positiveAnswer->setReasonerTerm(edbTerm);
	if (bindings->frame()) {
		positiveAnswer->setFrame(bindings->frame());
	}
	// add predicate groundings to the answer
	for (auto &rdfLiteral: q->path()) {
		auto p = rdfLiteral->predicate();
		auto p_instance = applyBindings(p, *positiveAnswer->substitution());
		positiveAnswer->addGrounding(
				std::static_pointer_cast<Predicate>(p_instance),
				bindings->frame(),
				rdfLiteral->isNegated());
	}
	return positiveAnswer;
}

AnswerPtr QueryableBackend::no(const GraphPathQueryPtr &q) {
	static const auto edbTerm = Atom::Tabled("EDB");
	// send one negative answer if no positive answer was found
	auto negativeAnswer = std::make_shared<AnswerNo>();
	negativeAnswer->setReasonerTerm(edbTerm);
	// the answer is uncertain as we only were not able to obtain a positive answer
	// which does not mean that there is no positive answer.
	negativeAnswer->setIsUncertain(true);
	// add ungrounded literals to negative answer.
	// but at the moment the information is lost at which literal the query failed.
	// TODO: would be great if we could report the failing literal.
	//       but seems hard to provide this information in the current framework.
	//       it could be queried here, but that seems a bit costly.
	// well at least we know if it is a single literal.
	if (q->path().size() == 1) {
		negativeAnswer->addUngrounded(q->path().front()->predicate(),
									  q->path().front()->isNegated());
	}
	return negativeAnswer;
}
