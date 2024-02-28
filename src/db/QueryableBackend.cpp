/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/db/QueryableBackend.h"
#include "knowrob/ThreadPool.h"
#include "knowrob/queries/AnswerNo.h"

using namespace knowrob;

AtomPtr QueryableBackend::versionProperty = IRIAtom::Tabled("tripledbVersionString");

std::vector<std::string> QueryableBackend::getOrigins() {
	static auto v_origin = std::make_shared<Variable>("&v");
	static auto v_version = std::make_shared<Variable>("&v");
	std::vector<std::string> origins;
	match(FramedTriplePattern(v_origin,versionProperty,v_version),
		  [&](const FramedTriple &triple) { origins.emplace_back(triple.subject()); });
	return origins;
}

void QueryableBackend::setVersionOfOrigin(std::string_view origin, std::string_view version) {
	FramedTripleView triple;
	triple.setSubject(origin);
	triple.setPredicate(versionProperty->stringForm());
	triple.setStringValue(version);
	insertOne(triple);
}

std::optional<std::string> QueryableBackend::getVersionOfOrigin(std::string_view origin) {
	static auto v_version = std::make_shared<Variable>("&v");
	std::optional<std::string> version;
	match(FramedTriplePattern(std::make_shared<Atom>(origin),versionProperty,v_version),
		  [&](const FramedTriple &triple) { version = triple.createStringValue(); });
	return version;
}

void QueryableBackend::evaluateQuery(const ConjunctiveQueryPtr &q, const TokenBufferPtr &resultStream) {
	static const auto edbTerm = Atom::Tabled("EDB");
	auto channel = TokenStream::Channel::create(resultStream);

	try {
		bool hasPositiveAnswer = false;
		query(q, [&channel,&hasPositiveAnswer](const AnswerPtr &answer) {
			channel->push(answer);
			hasPositiveAnswer = true;
		});

		if (!hasPositiveAnswer) {
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
			if (q->literals().size() == 1) {
				negativeAnswer->addUngrounded(q->literals().front()->predicate(),
											  q->literals().front()->isNegated());
			}
			channel->push(negativeAnswer);
		}
		channel->push(EndOfEvaluation::get());
	}
	catch (const std::exception &e) {
		// make sure EOS is pushed to the stream
		channel->push(EndOfEvaluation::get());
		throw;
	}
}

TokenBufferPtr QueryableBackend::submitQuery(const ConjunctiveQueryPtr &query) {
	std::shared_ptr<TokenBuffer> result = std::make_shared<TokenBuffer>();
	auto runner =
			std::make_shared<ThreadPool::LambdaRunner>([this, query, result](const ThreadPool::LambdaRunner::StopChecker&) {
				evaluateQuery(query, result);
			});
	DefaultThreadPool()->pushWork(runner, [result, query](const std::exception &e) {
		KB_WARN("an exception occurred for graph query ({}): {}.", *query, e.what());
		result->close();
	});
	return result;
}
