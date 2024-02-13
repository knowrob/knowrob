from knowrob import *
from typing import List
from langchain_openai import ChatOpenAI
from langchain.prompts.prompt import PromptTemplate
from langchain_core.output_parsers import StrOutputParser


TEMPLATE = """KnowRob is a software that accepts triple queries in the form of \"triple(Subject, Predicate, Object)\" and use an Prolog-like Semantic. 
Here are some examples of statements in natural language translated to triples:

The storing place of Milk is the fridge? is represented as triple(milk, has_storing_place, fridge)

The bread has the disposition of cutting is represented as triple(bread, has_disposition, cutting)

You will get an triple with one field unspecified as input, the not specified field will be filled with the variable Unspecified.

You should only answer with the filled in triple, for example for the input triple(milk, has_storing_place, Unspecified) you should answer
triple(milk, has_storing_place, fridge)

You are only allowed to answer with the filled triple.

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
		# configure the OpenAI API
		self.llm = ChatOpenAI(openai_api_key="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")

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
		description = PredicateDescription(indicator, PredicateType.BUILT_IN, MaterializationStrategy.ON_DEMAND)
		return description

	def submitQuery(self, query: RDFLiteral, ctx: QueryContext) -> TokenBuffer:
		# NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
		InputTriple = "triple(" + str(query.subjectTerm()) + ", " + str(query.propertyTerm()) + ", Unspecified)"
		print("InputTriple: " + InputTriple)

		prompt = CUSTOM_PROMPT
		output_parser = StrOutputParser()

		chain = prompt | self.llm | output_parser
		# print prompt
		answer = chain.invoke({"input": InputTriple})
		print("Answer: " + answer)

		# parse triple(Sub, Pred, Obj) to three string Sub, Pred, Obj
		answer = answer.replace("triple(", "").replace(")", "").replace(" ", "")
		answerList = answer.split(",")
		print(str(answerList))

		tokenBuffer = TokenBuffer()
		tokenChan = TokenChannel.create(tokenBuffer)

		answer = AnswerYes()
		answer.set(Variable("A"), StringTerm(answerList[2]))

		tokenChan.push(answer)
		tokenChan.push(EndOfEvaluation.get())

		return tokenBuffer
