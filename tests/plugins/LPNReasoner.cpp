#include "LPNReasoner.h"

#include "knowrob/storage/QueryableStorage.h"
#include "knowrob/semweb/GraphSequence.h"
#include "knowrob/semweb/GraphBuiltin.h"
#include "knowrob/terms/ListTerm.h"

using namespace knowrob;

LPNReasoner::LPNReasoner(std::string_view) : RDFGoalReasoner() {
    loves = IRIAtom::Tabled("http://knowrob.org/kb/lpn#loves");
    defineRelation(PredicateIndicator("http://knowrob.org/kb/lpn#jealous",2));
    defineRelation(PredicateIndicator("http://knowrob.org/kb/lpn#pos",2));
    defineRelation(PredicateIndicator("http://knowrob.org/kb/lpn#quaternion",2));
}

bool LPNReasoner::initializeReasoner(const PropertyTree&) {
    // nothing to do here
    return true;
}

bool LPNReasoner::evaluate(RDFGoalPtr goal) {
	// The goal is a conjunction that contains a single RDF literal of the form p(s, o).
	// There will not be any other literals in the goal as we have not enabled
	// the use of complex formulas in the reasoner configuration (via enableFeature/1).
	static const auto jealousAtom = IRIAtom::Tabled("http://knowrob.org/kb/lpn#jealous");
	static const auto posAtom = IRIAtom::Tabled("http://knowrob.org/kb/lpn#pos");
	static const auto quaternionAtom = IRIAtom::Tabled("http://knowrob.org/kb/lpn#quaternion");
	auto triplePattern = goal->rdfLiterals().at(0);

	if (*triplePattern->propertyTerm() == *posAtom) {
		return evaluatePos(goal, triplePattern);
	} else if (*triplePattern->propertyTerm() == *jealousAtom) {
		return evaluateJealous(goal, triplePattern);
	} else if (*triplePattern->propertyTerm() == *quaternionAtom) {
		return evaluateQuaternion(goal, triplePattern);
	} else {
		KB_ERROR("Unknown predicate: {}", *triplePattern->propertyTerm());
		return false;
	}
}

bool LPNReasoner::evaluateJealous(const RDFGoalPtr &goal, const std::shared_ptr<TriplePattern> &triplePattern) {
    TermPtr s = triplePattern->subjectTerm();
    TermPtr o = triplePattern->objectTerm();
    VariablePtr z = std::make_shared<Variable>("z");
    // Push a debug message to the logger of the knowledge base.
    KB_DEBUG("Checking if {} is jealous of {}.", *s, *o);
    // Create a query that checks if subj and obj both love the same person.
    // A builtin is used to ensure that subj and obj are different.
    auto query_term = std::make_shared<GraphSequence>(std::vector<std::shared_ptr<GraphTerm>>{
        std::make_shared<GraphPattern>(s, loves, z),
        std::make_shared<GraphPattern>(o, loves, z),
        GraphBuiltin::notEqual(s, o)
    });
    // Execute the query using the storage of the reasoner and call query.push for each solution.
    // A solution is represented as a dictionary of variable bindings that can be applied to the
    // query formula to replace variables with constants.
    auto queryableStorage = std::dynamic_pointer_cast<QueryableStorage>(storage());
    if (!queryableStorage) {
		KB_ERROR("Storage is not queryable.");
		return false;
	}
    queryableStorage->query(std::make_shared<GraphQuery>(query_term),
    	[&goal](const auto& solution) {
        	goal->push(solution);
        });
    return true;
}

bool LPNReasoner::evaluatePos(const RDFGoalPtr &goal, const std::shared_ptr<TriplePattern> &triplePattern) {
	TermPtr s = triplePattern->subjectTerm();
	TermPtr o = triplePattern->objectTerm();
	// Push a debug message to the logger of the knowledge base.
	KB_DEBUG("Checking position of {}.", *s);
	auto solution = std::make_shared<Bindings>();
	if (s->isVariable()) {
		solution->set(
			std::static_pointer_cast<Variable>(s),
			Atom::Tabled("foo"));
	}
	if (o->isVariable()) {
		auto zero = XSDAtomic::create("0.0", xsdTypeToIRI(XSDType::DOUBLE));
		solution->set(
			std::static_pointer_cast<Variable>(o),
			std::make_shared<ListTerm>(std::vector<TermPtr>{zero,zero,zero}));
	}
	goal->push(solution);
	return true;
}

bool LPNReasoner::evaluateQuaternion(const RDFGoalPtr &goal, const std::shared_ptr<TriplePattern> &triplePattern) {
	TermPtr s = triplePattern->subjectTerm();
	TermPtr o = triplePattern->objectTerm();
	// Push a debug message to the logger of the knowledge base.
	KB_DEBUG("Checking quaternion of {}.", *s);
	auto solution = std::make_shared<Bindings>();
	if (s->isVariable()) {
		solution->set(
			std::static_pointer_cast<Variable>(s),
			Atom::Tabled("foo"));
	}
	if (o->isVariable()) {
		auto zero = XSDAtomic::create("0.0", xsdTypeToIRI(XSDType::DOUBLE));
		solution->set(
			std::static_pointer_cast<Variable>(o),
			std::make_shared<ListTerm>(std::vector<TermPtr>{zero,zero,zero,zero}));
	}
	goal->push(solution);
	return true;
}

REASONER_PLUGIN(LPNReasoner, "LPNReasoner")
