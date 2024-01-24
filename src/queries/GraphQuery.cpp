//
// Created by daniel on 04.04.23.
//

#include "knowrob/queries/GraphQuery.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/modalities/PastModality.h"
#include "knowrob/Logger.h"
#include "knowrob/formulas/Negation.h"

using namespace knowrob;

GraphQuery::GraphQuery(const std::vector<RDFLiteralPtr> &literals, const QueryContextPtr &ctx)
: Query(ctx),
  literals_(literals)
{
    init();
}

GraphQuery::GraphQuery(const RDFLiteralPtr &literal, const QueryContextPtr &ctx)
: Query(ctx),
  literals_({literal})
{
    init();
}

void GraphQuery::init()
{
    std::vector<FormulaPtr> formulae(literals_.size());

    for(int i=0; i<literals_.size(); i++)
    {
        if(literals_[i]->isNegated()) {
            formulae[i] = std::make_shared<Negation>(literals_[i]->predicate());
        }
        else {
            formulae[i] = literals_[i]->predicate();
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

std::ostream& GraphQuery::print(std::ostream &os) const
{
    os << *formula();
    return os;
}
