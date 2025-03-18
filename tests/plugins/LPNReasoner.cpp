#include "LPNReasoner.h"

using namespace knowrob;

LPNReasoner::LPNReasoner() : RDFGoalReasoner() {
    loves = IRIAtom::Tabled("http://knowrob.org/kb/lpn#loves");
    defineRelation(PredicateIndicator("http://knowrob.org/kb/lpn#jealous",2));
}

bool LPNReasoner::initializeReasoner(const PropertyTree& config) {
    // nothing to do here
    return true;
}

bool LPNReasoner::evaluate(RDFGoalPtr goal) {
    // The goal is a conjunction that contains a single RDF literal of the form jealous(s, o).
    // There will not be any other literals in the goal as we have not enabled
    // the use of complex formulas in the reasoner configuration (via enableFeature/1).
    auto literal = goal->rdfLiterals().at(0);
    TermPtr s = literal->subjectTerm();
    TermPtr o = literal->objectTerm();
    // Push a debug message to the logger of the knowledge base.
    KB_DEBUG("Checking if {} is jealous of {}.", *s, *o);
    // Create a query that checks if subj and obj both love the same person.
    // A builtin is used to ensure that subj and obj are different.
    GraphSequence query_term({
        GraphPattern(s, loves, Variable("z")),
        GraphPattern(o, loves, Variable("z")),
        GraphBuiltin::notEqual(s, o)
    });
    // Execute the query using the storage of the reasoner and call query.push for each solution.
    // A solution is represented as a dictionary of variable bindings that can be applied to the
    // query formula to replace variables with constants.
    storage()->query(GraphQuery(query_term), [&goal](const auto& solution) {
        goal->push(solution);
    });
    return true;
}
