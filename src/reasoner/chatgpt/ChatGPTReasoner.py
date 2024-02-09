from knowrob import *
from typing import List
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate

TEMPLATE = """KnowRob is a software that accepts triple queries in the form of \"triple(Subject, Predicate, Object)\" and use an Prolog-like Semantic. 
Here are some examples of statements in natural language translated to triples:

The storing place of Milk is the fridge? is represented as triple(milk, has_storing_place, fridge)

The bread has the disposition of cutting is represented as triple(bread, has_disposition, cutting)

You will get an triple with one field unspecified as input, the not specified field will be filled with the variable Unspecified.

You should only answer with the filled in triple, for example for the input triple(milk, has_storing_place, Unspecified) you should answer
triple(milk, has_storing_place, fridge)

Give me the filled triple for the following input:
{input}"""

CUSTOM_PROMPT = PromptTemplate(
	input_variables=["input"], template=TEMPLATE
)

class ChatGPTReasoner(ReasonerWithBackend):
	"""
	A reasoner that uses the ChatGPT model to answer questions.
	"""

	def __init__(self):
		super(ChatGPTReasoner, self).__init__()
		logError("ChatGPTReasoner initialized")
		# configure the OpenAI API
		llm = OpenAI(openai_api_key="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")

	def loadConfig(self, config: ReasonerConfiguration) -> bool:
		return True

	def start(self):
		pass

	def stop(self):
		pass

	def insertOne(self, triple: StatementData) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		logWarn("insertOne not supported")
		return False

	def insertAll(self, triples: List[StatementData]) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		logWarn("insertAll not supported")
		return False

	def removeOne(self, triple: StatementData) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		return False

	def removeAll(self, triples: List[StatementData]) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		return False

	def removeMatching(self, query: RDFLiteral, do_match_many: bool) -> int:
		# NOTE: currently only the initial loading of OWL files is supported
		return 0

	def getDescription(self, indicator: PredicateIndicator) -> PredicateDescription:
		description = PredicateDescription(indicator=indicator)
		return description

	def submitQuery(self, query: RDFLiteral, ctx: QueryContext) -> TokenBuffer:
		# NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
		InputTriple = "triple(" + query.subjectTerm() + ", " + query.predicateTerm() + ", Unspecified)"
		logError("InputTriple: ", InputTriple)
		print("InputTriple: " + InputTriple, file=stderr)

		prompt = CUSTOM_PROMPT.format(input=InputTriple)
		# print prompt
		answer = llm.predict(prompt)

		tokenBuffer = TokenBuffer()
		token = Token()
		token.type = TokenType.STRING
		token.value = answer
		tokenBuffer.tokens = [token]

		return tokenBuffer
