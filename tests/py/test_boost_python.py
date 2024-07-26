try:
	# This case works in ros1 environments
	from knowrob.kb import *
except ImportError:
	# If the import fails, import the knowrob.so directly
	from knowrob import *

def perform_query(settings_path, query_string,):
	# Load the settings
	modalities = {
		"epistemicOperator": 0,
		"aboutAgentIRI": "",
		"confidence": 0.0,
		"temporalOperator": 0,
		"minPastTimestamp": -1.0,
		"maxPastTimestamp": -1.0,
	}
	# Load the settings
	kb = KnowledgeBase(settings_path)
	# Create a formula for the query
	phi = QueryParser.parse(query_string)
	# Apply the modality
	mPhi = applyModality(modalities, phi)
	# Get Result Stream
	resultStream = kb.submitQueryFormula(mPhi, QueryContext(QueryFlag.QUERY_FLAG_ALL_SOLUTIONS))
	resultQueue = resultStream.createQueue()
	# Get the result
	nextResult = resultQueue.pop_front()
	return nextResult


def atom_to_python(term):
	# make some tests that the class hierarchy is correct
	# NOTE: unfortunately, checking the type is not enough. There are runtime conversions done
	#       that could fail. So it makes sense to test the mapping more thoroughly for different mapped types.
	assert isinstance(term, Term), "argument is not a Term"
	assert isinstance(term, Atom), "argument is not an Atom"
	# check for the existence of some methods
	assert hasattr(term, "stringForm"), "term has no stringForm method"
	assert hasattr(term, "variables"), "term has no variables method"
	assert hasattr(term, "atomType"), "term has no atomType method"
	assert hasattr(term, "atomicType"), "term has no atomicType method"
	assert hasattr(term, "termType"), "term has no termType method"
	assert hasattr(term, "isAtomic"), "term has no isAtomic method"
	# check that some boolean methods return the right value
	assert term.isAtomic(), "term is not atomic"
	assert term.isAtom(), "term is not an atom"
	assert not term.isVariable(), "term is a variable"
	assert not term.isFunction(), "term is a function"
	assert not term.isNumeric(), "term is numeric"
	assert not term.isString(), "term is a string"
	# check that some enums can be accessed
	assert type(term.atomType()) == AtomType, "atomType is not an AtomType"
	assert term.atomType() == AtomType.REGULAR, "atomType is not REGULAR"
	assert type(term.atomicType()) == AtomicType, "atomicType is not an AtomicType"
	assert term.atomicType() == AtomicType.ATOM, "atomicType is not ATOM"
	assert type(term.termType()) == TermType, "termType is not a TermType"
	assert term.termType() == TermType.ATOMIC, "termType is not ATOMIC"
	# test that string view can be mapped to Python string
	assert type(term.stringForm()) == str, "stringForm is not a string"


def downcast_term_to_atom(term):
	# test that a Term can be downcasted to an Atom
	assert isinstance(term, Term), "argument is not a Term"
	assert isinstance(term, FramedTriple), "argument is not a FramedTriple"


def modify_triple_in_python(triple):
	assert isinstance(triple, FramedTriple), "argument is not a FramedTriple"
	assert hasattr(triple, "setSubject"), "term has no setSubject method"
	assert hasattr(triple, "setPredicate"), "term has no setPredicate method"
	triple.setSubject("olleh")
	triple.setPredicate("swonk")


def string_copy_from_python():
	# test that string can be copied from Python to C++.
	# here the constructor actually takes a string_view argument which is then copied internally.
	term = String("hello")
	assert term is not None, "term is None"
	assert term.stringForm() == "hello", "stringForm is not 'hello'"
	return term


def optional_is_none(optional):
	assert optional is None, "optional is not None"


def optional_is_not_none(optional):
	assert optional is not None, "optional is None"


def set_xsd_optional(optional):
	assert optional == XSDType.STRING, "optional is not STRING"
	optional = XSDType.DOUBLE
	assert optional == XSDType.DOUBLE, "optional is not DOUBLE"


def connective_formulas():
	phi = QueryParser.parse("x & (y | z)")
	assert isinstance(phi, Conjunction), "argument is not an Conjunction"
	args = phi.formulae()
	arg0 = args[0]
	assert isinstance(arg0, Predicate), "argument is not an Predicate"
	arg1 = args[1]
	assert isinstance(arg1, Disjunction), "argument is not an Conjunction"


def answer_queue():
	yes = AnswerYes()
	# push "yes" to the queue
	tokenQueue = TokenQueue()
	channel = TokenChannel.create(tokenQueue)
	channel.push(yes)
	# pop "yes" from the queue
	nextResult = tokenQueue.pop_front()
	# Check if the result is an posititve answer
	assert nextResult.tokenType() == TokenType.ANSWER_TOKEN
	assert nextResult.isPositive()
	assert isinstance(nextResult, AnswerYes), "argument is not an AnswerYes"
	# Check if the substitution is empty
	assert nextResult.substitution().empty()

def kb_positive_query(settings_path):
	# Test that a query returning "yes" can be made
	nextResult = perform_query(settings_path, "swrl_test:hasAncestor(swrl_test:'Lea', ?y)")
	# Check if the result is a positive answer
	assert nextResult.tokenType() == TokenType.ANSWER_TOKEN
	assert nextResult.isPositive()
	assert isinstance(nextResult, AnswerYes), "argument is not an AnswerYes"
	# Check if the substitution is not empty
	assert not nextResult.substitution().empty()
	# Get result
	for substitution in nextResult.substitution():
		term = substitution[2]
		assert term.termType() == TermType.ATOMIC
		assert term.atomicType() == AtomicType.ATOM or term.atomicType() == AtomicType.STRING
		stringResult = term.humanReadableForm()
		assert stringResult == 'swrl_test:Fred', "Result is not 'swrl_test:Fred'"

def kb_negative_query(settings_path):
	# Test that a query returning "no" can be made
	nextResult = perform_query(settings_path, "swrl_test:hasAncestor(swrl_test:'Lea', swrl_test:'Lea')")
	# Check if the result is a negative answer
	assert nextResult.tokenType() == TokenType.ANSWER_TOKEN
	assert isinstance(nextResult, AnswerNo), "argument is not an AnswerNo"
	assert nextResult.isNegative()

def kb_dont_know_query(settings_path):
	# Test that a query returning "don't know" can be made
	nextResult = perform_query(settings_path, "r(?x, ?y)")
	# Check if the result is an AnswerDontKnow
	assert nextResult.tokenType() == TokenType.ANSWER_TOKEN
	assert isinstance(nextResult, AnswerDontKnow), "argument is not an AnswerDontKnow"
	assert not nextResult.isPositive()
	assert not nextResult.isNegative()

def kb_assert(settings_path):
	# Test that a assertion to the knowledge base can be made
	kb = KnowledgeBase(settings_path)
	# Create a triple
	triple = FramedTripleCopy("swrl_test:Dieter", "swrl_test:hasAncestor", "swrl_test:Friedhelm")
	# Assert the triple
	kb.insertOne(triple)
	# Query the triple
	nextResult = perform_query(settings_path, "swrl_test:hasAncestor(swrl_test:Dieter, swrl_test:Friedhelm)")
	# Check if the result is a positive answer
	assert nextResult.tokenType() == TokenType.ANSWER_TOKEN
	assert nextResult.isPositive()



