//
// Created by daniel on 04.04.23.
//

#include "knowrob/queries/GraphQuery.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/Logger.h"

using namespace knowrob;

GraphQuery::GraphQuery(
        const std::vector<LiteralPtr> &literals,
        int flags,
        ModalityFrame modalFrame)
: literals_(literals),
  modalFrame_(std::move(modalFrame)),
  flags_(flags)
{}

GraphQuery::GraphQuery(LiteralPtr &literal, int flags, ModalityFrame modalFrame)
: literals_({literal}),
  modalFrame_(std::move(modalFrame)),
  flags_(flags)
{}

/**
            auto literals = queryData->graphQuery_->literals();
            // TODO: at least need to group into negative and positive literals here.
            //       also would make sense to sort according to number of variables
            std::vector<FormulaPtr> formulae(literals.size());
            int counter=0;
            for(auto &l : literals) {
                // FIXME: handle negative literals
                formulae[counter++] = l->predicate();
            }
            auto phi = std::make_shared<Conjunction>(formulae);
 */

std::list<semweb::FramedLiteral> GraphQuery::asTripleExpression()
{
    std::list<semweb::FramedLiteral> out;
    auto expressions = std::list<semweb::FramedLiteral>();

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
                    expr.setBeginOperator(semweb::FramedLiteral::LT);
                    expr.setBeginTerm(std::make_shared<DoubleTerm>(TimePoint::now().value()));
                }
                else if(pastOperator->isModalNecessity()) {
                    // include only triples that were always true in the past
                    // TODO: reconsider how this case should be encoded in triple expressions
                    expr.setBeginOperator(semweb::FramedLiteral::LEQ);
                    expr.setBeginTerm(std::make_shared<DoubleTerm>(0.0));
                    expr.setEndOperator(semweb::FramedLiteral::GEQ);
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
