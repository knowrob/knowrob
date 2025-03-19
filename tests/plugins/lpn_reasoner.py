from knowrob import *


class LPNReasoner(RDFGoalReasoner):
	def __init__(self):
		super(LPNReasoner, self).__init__()
		self.loves = IRIAtom("http://knowrob.org/kb/lpn#loves")
		# The reasoner defines a new predicate lpn:jealous that can be evaluated by the reasoner
		self.defineRelation(IRIAtom("http://knowrob.org/kb/lpn#jealous"))

	def initializeReasoner(self, config: PropertyTree) -> bool:
		# nothing to do here
		return True

	def evaluate(self, goal: RDFGoal) -> bool:
		# The goal is a conjunction that contains a single RDF literal of the form jealous(s, o).
		# There will not be any other literals in the goal as we have not enabled
		# the use of complex formulas in the reasoner configuration (via enableFeature/1).
		literal = goal.rdfLiterals()[0]
		s : Term = literal.subjectTerm()
		o : Term = literal.objectTerm()
		# Push a debug message to the logger of the knowledge base.
		logDebug("Checking if %s is jealous of %s" % (s, o))
		# Create a query that checks if subj and obj both love the same person.
		# A builtin is used to ensure that subj and obj are different.
		query_term = GraphSequence([
			GraphPattern(s, self.loves, Variable("z")),
			GraphPattern(o, self.loves, Variable("z")),
			GraphBuiltin.notEqual(s, o)])
		# Execute the query using the storage of the reasoner and call query.push for each solution.
		# A solution is represented as a dictionary of variable bindings that can be applied to the
		# query formula to replace variables with constants.
		self.storage().query(GraphQuery(query_term), goal.push)
		return True
