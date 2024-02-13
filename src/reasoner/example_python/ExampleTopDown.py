from knowrob import *
from typing import List

class ExampleTopDown(ReasonerWithBackend):
    """
    A reasoner that uses the ChatGPT model to answer questions.
    """

    def __init__(self):
        super(ExampleTopDown, self).__init__()

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
        InputTriple = "triple(" + str(query.subjectTerm()) + ", " + str(query.propertyTerm()) + ", Unspecified)"
        print("InputTriple: " + InputTriple)

        # This example reasoner alway returns the triple
        # triple(test, test, test)
        # if one of the two fields are bound with test
        # and the other is unbound


        tokenBuffer = TokenBuffer()
        tokenChan = TokenChannel.create(tokenBuffer)

        answer = AnswerYes()
        answer.set(Variable("A"), StringTerm("test"))

        tokenChan.push(answer)
        tokenChan.push(EndOfEvaluation.get())

        return tokenBuffer
