/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/GraphQuery.h"
#include "knowrob/triples/GraphPattern.h"
#include "knowrob/triples/GraphSequence.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/triples/GraphUnion.h"
#include "knowrob/formulas/Conjunction.h"

using namespace knowrob;

GraphQuery::GraphQuery(std::shared_ptr<GraphTerm> &queryTerm, const QueryContextPtr &ctx)
		: Query(ctx),
		  term_(queryTerm) {
}

GraphQuery::GraphQuery(const FramedTriplePatternPtr &query, const QueryContextPtr &ctx)
		: Query(ctx),
		  term_(std::make_shared<GraphPattern>(query)) {
}

GraphQuery::GraphQuery(const std::vector<FramedTriplePatternPtr> &query, const QueryContextPtr &ctx)
		: Query(ctx),
		  term_(std::make_shared<GraphSequence>()) {
	auto sequence = std::static_pointer_cast<GraphSequence>(term_);
	for (auto &p: query) {
		sequence->addPattern(p);
	}
}

static FormulaPtr toFormula_recursive(GraphTerm *term) { //NOLINT
	switch (term->termType()) {
		case GraphTermType::Pattern: {
			auto pattern = ((GraphPattern *) term)->value();
			if (pattern->isNegated()) {
				return std::make_shared<Negation>(pattern->predicate());
			} else {
				return pattern->predicate();
			}
		}
		case GraphTermType::Union: {
			auto &terms = ((GraphUnion *) term)->terms();
			std::vector<FormulaPtr> formulae;
			for (const auto &t: terms) {
				auto phi = toFormula_recursive(t.get());
				if (phi) {
					formulae.push_back(phi);
				}
			}
			return std::make_shared<Disjunction>(formulae);
		}
		case GraphTermType::Sequence: {
			auto &terms = ((GraphSequence *) term)->terms();
			std::vector<FormulaPtr> formulae;
			for (const auto &t: terms) {
				auto phi = toFormula_recursive(t.get());
				if (phi) {
					formulae.push_back(phi);
				}
			}
			return std::make_shared<Conjunction>(formulae);
		}
		case GraphTermType::Builtin:
			break;
	}
	return nullptr;
}

FormulaPtr GraphQuery::toFormula() const {
	return toFormula_recursive(term_.get());
}

void GraphQuery::write(std::ostream &os) const {
	os << *term_;
}
