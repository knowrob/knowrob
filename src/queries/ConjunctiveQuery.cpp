/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/ConjunctiveQuery.h"
#include "knowrob/formulas/Negation.h"
#include "knowrob/formulas/Conjunction.h"

using namespace knowrob;

ConjunctiveQuery::ConjunctiveQuery(const std::vector<FramedTriplePatternPtr> &literals, const QueryContextPtr &ctx)
: Query(ctx),
  literals_(literals)
{
    init();
}

ConjunctiveQuery::ConjunctiveQuery(const FramedTriplePatternPtr &literal, const QueryContextPtr &ctx)
: Query(ctx),
  literals_({literal})
{
    init();
}

void ConjunctiveQuery::init()
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

const FormulaPtr& ConjunctiveQuery::formula() const
{
    return formula_;
}

std::ostream& ConjunctiveQuery::print(std::ostream &os) const
{
    os << *formula();
    return os;
}
