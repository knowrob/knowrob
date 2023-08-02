//
// Created by daniel on 04.04.23.
//

#include "knowrob/queries/GraphQuery.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/Logger.h"
#include "knowrob/formulas/Negation.h"

using namespace knowrob;

GraphQuery::GraphQuery(const std::vector<LiteralPtr> &literals, int flags, ModalityFrame modalFrame)
: Query(flags),
  literals_(literals),
  framedLiterals_(literals.size()),
  modalFrame_(std::move(modalFrame))
{
    init();
}

GraphQuery::GraphQuery(LiteralPtr &literal, int flags, ModalityFrame modalFrame)
: Query(flags),
  literals_({literal}),
  framedLiterals_(1),
  modalFrame_(std::move(modalFrame))
{
    init();
}

GraphQuery::GraphQuery(const PredicatePtr &predicate, int flags, ModalityFrame modalFrame)
: Query(flags),
  literals_({std::make_shared<Literal>(predicate, false)}),
  framedLiterals_(1),
  modalFrame_(std::move(modalFrame))
{
    init();
}

void GraphQuery::init()
{
    std::vector<FormulaPtr> formulae(literals_.size());

    for(int i=0; i<literals_.size(); i++)
    {
        framedLiterals_[i] = std::make_shared<FramedRDFLiteral>(literals_[i], modalFrame_);
        if(literals_[i]->isPositive()) {
            formulae[i] = literals_[i]->predicate();
        }
        else {
            formulae[i] = std::make_shared<Negation>(literals_[i]->predicate());
        }
    }

    if(literals_.size()==1) {
        formula_ = formulae[0];
    }
    else {
        formula_ = std::make_shared<Conjunction>(formulae);
    }
}

const FormulaPtr& GraphQuery::formula() const
{
    return formula_;
}

const ModalityFrame& GraphQuery::modalFrame() const
{
    return modalFrame_;
}

std::ostream& GraphQuery::print(std::ostream &os) const
{
    // TODO: also print ModalFrame
    os << *formula();
    return os;
}
