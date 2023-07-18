//
// Created by daniel on 04.04.23.
//

#include "knowrob/semweb/GraphQuery.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/Logger.h"

using namespace knowrob;

std::list<semweb::TripleExpression> GraphQuery::asTripleExpression()
{
    std::list<semweb::TripleExpression> out;
    auto expressions = std::list<semweb::TripleExpression>();

    auto epistemicOperator = modalFrame_.epistemicOperator();
    auto pastOperator = modalFrame_.pastOperator();

    for(auto &lit : literals())
    {
        auto &expr = expressions.emplace_back(lit->predicate());

        if(epistemicOperator) {
            auto epistemicModality = (const EpistemicModality*) &epistemicOperator->modality();
            auto o_agent = epistemicModality->agent();
            if(o_agent) {
                expr.setAgentTerm(o_agent.value());
            }
            if(epistemicOperator->isModalPossibility()) {
                expr.setMinConfidence(0.0);
            }
        }

        if(pastOperator) {
            auto pastModality = (const PastModality*) &pastOperator->modality();
            if(pastModality) {
                // TODO: allow quantified time intervals in past modality
                if(pastOperator->isModalPossibility()) {
                    // include all triples before the current time point
                    expr.setBeginOperator(semweb::TripleExpression::LT);
                    expr.setBeginTerm(std::make_shared<DoubleTerm>(TimePoint::now().value()));
                }
                else if(pastOperator->isModalNecessity()) {
                    // include only triples that were always true in the past
                    // TODO: reconsider how this case should be encoded in triple expressions
                    expr.setBeginOperator(semweb::TripleExpression::LEQ);
                    expr.setBeginTerm(std::make_shared<DoubleTerm>(0.0));
                    expr.setEndOperator(semweb::TripleExpression::GEQ);
                    expr.setEndTerm(std::make_shared<DoubleTerm>(TimePoint::now().value()));
                }
            }
            else {
                KB_WARN("unexpected temporal operator in graph query!");
            }
        }
    }

    return out;
}
