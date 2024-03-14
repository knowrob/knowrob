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
	// Expand the query. Currently, this is mainly used to compute the answer frame here.
	// But also to insert some builtins for occasional triples.
	ExpansionContext exp_ctx;
	exp_ctx.query_ctx = q->ctx();
	auto expandedQuery = expand(exp_ctx, q);

	auto channel = TokenStream::Channel::create(resultStream);
	try {
		bool hasPositiveAnswer = false;
		query(expandedQuery, [&](const BindingsPtr &bindings) {
			auto answer = yes(q, bindings);

			// The answer is uncertain if any of the groundings is uncertain.
			answer->setIsUncertain(std::any_of(exp_ctx.u_vars.begin(), exp_ctx.u_vars.end(), [&](const VariablePtr &v) {
				auto &u_v = bindings->get(v->name());
				return (u_v && u_v->isNumeric() && std::static_pointer_cast<Numeric>(u_v)->asBoolean());
			}), std::nullopt);

			// The answer is occasional if any of the groundings has occasional=true flag
			answer->setIsOccasionallyTrue(
					std::any_of(exp_ctx.o_vars.begin(), exp_ctx.o_vars.end(), [&](const VariablePtr &v) {
						auto &o_v = bindings->get(v->name());
						return (o_v && o_v->isNumeric() && std::static_pointer_cast<Numeric>(o_v)->asBoolean());
					}));

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

std::shared_ptr<AnswerYes> QueryableBackend::yes(const GraphPathQueryPtr &q, const BindingsPtr &bindings) {
	static const auto edbTerm = Atom::Tabled("EDB");

	auto positiveAnswer = std::make_shared<AnswerYes>(bindings);
	// Indicate that EDB has computed the grounding.
	positiveAnswer->setReasonerTerm(edbTerm);
	// Apply query context to the answer for some parameters.
	positiveAnswer->applyFrame(q->ctx()->selector);

	// Add predicate groundings to the answer
	for (auto &rdfLiteral: q->path()) {
		auto p = rdfLiteral->predicate();
		auto p_instance = applyBindings(p, *positiveAnswer->substitution());
		positiveAnswer->addGrounding(
				std::static_pointer_cast<Predicate>(p_instance),
				positiveAnswer->frame(),
				rdfLiteral->isNegated());
	}

	return positiveAnswer;
}

std::shared_ptr<AnswerNo> QueryableBackend::no(const GraphPathQueryPtr &q) {
	static const auto edbTerm = Atom::Tabled("EDB");

	// send one negative answer if no positive answer was found
	auto negativeAnswer = std::make_shared<AnswerNo>();
	negativeAnswer->setReasonerTerm(edbTerm);
	// Apply query context "origin" and "perspective" to the answer if any
	negativeAnswer->applyFrame(q->ctx()->selector);
	// the answer is uncertain as we only were not able to obtain a positive answer
	// which does not mean that there is no positive answer.
	negativeAnswer->setIsUncertain(true, std::nullopt);

	// Add ungrounded literals to negative answer.
	// But at the moment the information is not provided by EDBs, would be difficult to implement e.g. in MongoDB.
	// Well at least we know if the query is only a single triple pattern.
	// TODO: It would be great if we could report the failing triple pattern.
	//       It could be queried here, but that seems a bit costly just to have this info.
	if (q->path().size() == 1) {
		negativeAnswer->addUngrounded(q->path().front()->predicate(),
									  q->path().front()->isNegated());
	}
	return negativeAnswer;
}

GraphQueryPtr QueryableBackend::expand(const GraphQueryPtr &q) {
	ExpansionContext exp_ctx;
	exp_ctx.query_ctx = q->ctx();
	return expand(exp_ctx, q);
}

GraphQueryPtr QueryableBackend::expand(ExpansionContext &ctx, const GraphQueryPtr &q) {
	// Initialize begin/end variables for the computation of the time interval.
	static const auto var_begin = std::make_shared<Variable>("_begin");
	static const auto var_end = std::make_shared<Variable>("_end");
	ctx.accumulated_begin = var_begin;
	ctx.accumulated_end = var_end;

	// Expand the query
	auto expandedTerm = expand(ctx, q->term());

	// If the query uses SOMETIMES operator, prepend initialization of the accumulated_begin and accumulated_end variables
	// used to compute the intersection of triple time intervals.
	if (ctx.query_ctx->selector.occasional) {
		double b_min = ctx.query_ctx->selector.begin.value_or(0.0);
		double e_max = ctx.query_ctx->selector.end.value_or(std::numeric_limits<double>::max());
		auto set_b = GraphBuiltin::bind(var_begin, std::make_shared<Double>(b_min));
		auto set_e = GraphBuiltin::bind(var_end, std::make_shared<Double>(e_max));

		if (expandedTerm->termType() == GraphTermType::Sequence) {
			// Prepend to existing sequence
			auto seq_terms = std::static_pointer_cast<GraphSequence>(expandedTerm)->terms();
			seq_terms.insert(seq_terms.begin(), set_e);
			seq_terms.insert(seq_terms.begin(), set_b);
			expandedTerm = std::make_shared<GraphSequence>(seq_terms);
		} else {
			// Create a new sequence
			auto seq = std::make_shared<GraphSequence>();
			seq->addMember(set_b);
			seq->addMember(set_e);
			seq->addMember(expandedTerm);
			expandedTerm = seq;
		}
	}

	if (expandedTerm == q->term()) {
		return q;
	} else {
		return std::make_shared<GraphQuery>(expandedTerm, q->ctx());
	}
}

std::shared_ptr<GraphTerm>
QueryableBackend::expand(ExpansionContext &ctx, const std::shared_ptr<GraphTerm> &q) { // NOLINT
	switch (q->termType()) {
		case GraphTermType::Pattern:
			return expandPattern(ctx, std::static_pointer_cast<GraphPattern>(q));
		case GraphTermType::Sequence: {
			auto seq = std::static_pointer_cast<GraphSequence>(q);
			std::vector<std::shared_ptr<GraphTerm>> expandedTerms(seq->terms().size());
			if (expandAll(ctx, seq, expandedTerms)) {
				return std::make_shared<GraphSequence>(expandedTerms);
			}
			break;
		}
		case GraphTermType::Union: {
			auto unionTerm = std::static_pointer_cast<GraphUnion>(q);
			std::vector<std::shared_ptr<GraphTerm>> expandedTerms(unionTerm->terms().size());
			if (expandAll(ctx, unionTerm, expandedTerms)) {
				return std::make_shared<GraphUnion>(expandedTerms);
			}
			break;
		}
		case GraphTermType::Builtin:
			break;
	}
	return q;
}

bool QueryableBackend::expandAll(ExpansionContext &ctx, const std::shared_ptr<GraphConnective> &q, // NOLINT
								 std::vector<std::shared_ptr<GraphTerm>> &expandedTerms) {
	bool hasExpansion = false;
	for (size_t i = 0; i < q->terms().size(); ++i) {
		auto expandedTerm = expand(ctx, q->terms()[i]);
		expandedTerms[i] = expandedTerm;
		hasExpansion = hasExpansion || (expandedTerm != q->terms()[i]);
	}
	return hasExpansion;
}

std::shared_ptr<GraphTerm>
QueryableBackend::expandPattern(ExpansionContext &ctx, const std::shared_ptr<GraphPattern> &q) {
	const auto &p = q->value();
	ctx.counter += 1;

	// Find out if an additional variable must be added for the "isUncertain" flag.
	// If a variable exists, rather use the existing one.
	bool needsUncertainVar = false;
	if (p->isUncertainTerm()) {
		if (**p->isUncertainTerm() == *Numeric::trueAtom()) {
			// For each possibly uncertain triple, add a variable to the query for the isUncertain flag.
			needsUncertainVar = true;
		} else if (p->isUncertainTerm()->isVariable()) {
			// In case uncertain term is already a variable, remember it.
			ctx.u_vars.emplace_back(std::static_pointer_cast<Variable>(*p->isUncertainTerm()));
		}
	}

	// Find out if an additional variable must be added for the "isOccasional" flag.
	// If a variable exists, rather use the existing one.
	bool needsOccasionalVar = false;
	if (p->isOccasionalTerm()) {
		if (**p->isOccasionalTerm() == *Numeric::trueAtom()) {
			// For each possibly occasional triple, add a variable to the query for the isOccasional flag.
			needsOccasionalVar = true;
		} else if (p->isOccasionalTerm()->isVariable()) {
			// In case occasional term is already a variable, remember it.
			ctx.o_vars.emplace_back(std::static_pointer_cast<Variable>(*p->isOccasionalTerm()));
		}
	}

	// Find out if begin/end variables must be added for the computation of the time interval.
	// This is always the case if the query uses SOMETIMES operator.
	bool needsIntervalComputation = (ctx.query_ctx->selector.occasional);

	bool needsRewrite = needsUncertainVar || needsOccasionalVar || needsIntervalComputation;
	if (!needsRewrite) return q;

	auto pat_expanded = std::make_shared<FramedTriplePattern>(*p);

	if (needsUncertainVar) {
		static const std::string varPrefix = "_uncertain";
		// insert a fresh variable for the isUncertain flag
		auto u_var = std::make_shared<Variable>(varPrefix + std::to_string(ctx.counter));
		pat_expanded->setIsUncertainTerm(groundable<Numeric>(u_var));
		// also remember the variable in the context
		ctx.u_vars.emplace_back(u_var);
	}

	if (needsOccasionalVar) {
		static const std::string varPrefix = "_occasional";
		// insert a fresh variable for the isOccasional flag
		auto o_var = std::make_shared<Variable>(varPrefix + std::to_string(ctx.counter));
		pat_expanded->setIsOccasionalTerm(groundable<Numeric>(o_var));
		// also remember the variable in the context
		ctx.o_vars.emplace_back(o_var);
	}

	VariablePtr triple_begin, triple_end;
	if (needsIntervalComputation) {
		// Add begin/end variables possibly replacing fixed values for begin/end in the query.
		// This is ok assuming each pattern has the same query frame.
		// FILTER will be performed through a builtin instead, so deleting the values is ok,
		// SPARQL would insert ad-hoc variables anyway.
		auto g_begin = p->beginTerm();
		auto g_end = p->endTerm();

		if (g_begin.has_variable()) {
			triple_begin = g_begin.variable();
		} else {
			static const std::string varPrefix = "_begin";
			triple_begin = std::make_shared<Variable>(varPrefix + std::to_string(ctx.counter));
			pat_expanded->setBeginTerm(groundable<Double>(triple_begin));
		}

		if (g_end.has_variable()) {
			triple_end = g_end.variable();
		} else {
			static const std::string varPrefix = "_end";
			triple_end = std::make_shared<Variable>(varPrefix + std::to_string(ctx.counter));
			pat_expanded->setEndTerm(groundable<Double>(triple_end));
		}
	}

	auto q_expanded = std::make_shared<GraphPattern>(pat_expanded);
	std::shared_ptr<GraphTerm> outer_term = q_expanded;

	// If the query uses SOMETIMES operator, then we need to insert builtins to the query for
	// the computation of the time interval.
	if (needsIntervalComputation) {
		auto seq = std::make_shared<GraphSequence>();
		seq->addMember(q_expanded);

		// FILTER all triples that do not intersect with begin/end of query frame
		if (ctx.query_ctx->selector.begin) {
			auto ctx_begin = std::make_shared<Double>(ctx.query_ctx->selector.begin.value());
			seq->addMember(GraphBuiltin::lessOrEqual(ctx_begin, triple_end));
		}
		if (ctx.query_ctx->selector.end) {
			auto ctx_end = std::make_shared<Double>(ctx.query_ctx->selector.end.value());
			seq->addMember(GraphBuiltin::greaterOrEqual(ctx_end, triple_begin));
		}

		// Set accumulated begin and end time.
		// But not all backends support variable re-assignment.
		// For the ones that don't support it like (SPARQL), we need to introduce a new variable for each intersection computed.
		VariablePtr next_i_begin, next_i_end;
		if (supportsReAssignment()) {
			next_i_begin = ctx.accumulated_begin;
			next_i_end = ctx.accumulated_end;
		} else {
			static const std::string varPrefix_begin = "_i_begin";
			static const std::string varPrefix_end = "_i_end";
			next_i_begin = std::make_shared<Variable>(varPrefix_begin + std::to_string(ctx.counter));
			next_i_end = std::make_shared<Variable>(varPrefix_end + std::to_string(ctx.counter));
		}
		seq->addMember(GraphBuiltin::max(next_i_begin, ctx.accumulated_begin, triple_begin));
		seq->addMember(GraphBuiltin::min(next_i_end, ctx.accumulated_end, triple_end));
		ctx.accumulated_begin = next_i_begin;
		ctx.accumulated_end = next_i_end;

		// Ensure that begin < end
		seq->addMember(GraphBuiltin::less(ctx.accumulated_begin, ctx.accumulated_end));

		// Update outer term
		outer_term = seq;
	}

	return outer_term;
}
