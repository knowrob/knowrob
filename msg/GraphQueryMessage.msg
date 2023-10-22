# Languages in which graph queries can be encoded.
uint8 LANG_FOL=0 # Prolog-like syntax, @see ....

# Epistemic operators distinguishing knowledge and belief.
# Note that the operators can be indexed by an agent
# meaning that another agent has the knowledge or belief.
uint8 KNOWLEDGE=0 # "it is certain that ..."
uint8 BELIEF=1    # "it could be that ..."

# Temporal operators used to query truth
# of statements within time intervals.
uint8 CURRENTLY=0 # "it is now the case that ..."
uint8 ALL_PAST=1  # "it always was that ..."
uint8 SOME_PAST=2 # "it was the case that ..."

# Used to indicate that a timestamp remains unspecified.
float64 UNSPECIFIED_TIMESTAMP=-1.0

# Configure the language in which the graph query is encoded.
string lang # Default: LANG_FOL

# A string encoding a graph query that is to be evaluated
# wrt. the knowledge graph selected by the modality configured
# in this message.
# The query must by valid expression in the language configured
# via the "lang" field.
string queryString

# Determines the epistemic modality which is concerned
# with distinguishing knowledge (certain) from belief (uncertain).
# Defaults to KNOWLEDGE modality.
uint8 epistemicOperator # Default: KNOWLEDGE

# The IRI of another agent (i.e. not the controlled agent).
# Used to query whether another agent might has some knowledge
# or belief. This is thought to be used in HRI and multirobot
# scenarios where the robot estimates knowledge and belief of
# another agent.
# Defaults to empty string indicating that the query is about
# the agent's own knowledge or belief.
# Cannot be set in parallel with the aboutSimulationIRI parameter
string aboutAgentIRI # Default: ""

# The IRI to an KG of a mental simulation. 
# The IRI will be returned as the output of an MentalSimulation
# Action. This is thought to be used in mental emulation
# and also visual anticipation tasks.
# Defaults to empty string indicating that the query is about
# the agen'ts own knowledge or beliefstate.
# Cannot be set in parallel with the aboutAgenIRI parameter
string aboutSimulationIRI # Default: ""

# Determines the temporal modality of the query.
# Temporal modalities are used to cope with characteristics
# that change over time.
uint8 temporalOperator # Default: CURRENTLY

# The time interval considered by the past operator can be constrained
# by min/max timestamp parameters.
# If minPastTimestamp>0, then statements are only considered that are
# true after the time indicated by minPastTimestamp. Similarly for maxPastTimestamp.
# It is also allowed to set only one of the fields to a positive value.
float64 minPastTimestamp # Default: UNSPECIFIED_TIMESTAMP
float64 maxPastTimestamp # Default: UNSPECIFIED_TIMESTAMP

# The minimal confidence of the statement
float64 confidence # Default: 0.0