from knowrob import *

class KnowledgeApplication:
	def __init__(self, settings_path):
		self.kb = KnowledgeBase(settings_path)

	def query_one_solution(self, query_string):
		# Create a formula for the query
		phi = QueryParser.parse(query_string)
		logWarn("query: " + str(phi))
		# Get Result Stream
		resultStream = self.kb.submitQuery(phi, QueryContext(QueryFlag.QUERY_FLAG_ALL_SOLUTIONS))
		# Construct a queue from the stream to read its output
		resultQueue = resultStream.createQueue()
		# Get the result
		return resultQueue.pop_front()

	def assert_triple(self, subject, predicate, object):
		# Create a triple
		triple = TripleCopy(subject, predicate, object)
		# Assert the triple
		self.kb.insertOne(triple)

def ka_assert():
	# Initialize the KnowledgeApplication class, and assert a triple
	ka_settings_path = "tests/settings/test-application.json"
	ka = KnowledgeApplication(ka_settings_path)
	ka.assert_triple("http://knowrob.org/kb/lpn#hello",
					 "http://knowrob.org/kb/lpn#loves",
					 "http://knowrob.org/kb/lpn#world")
	# Test that we can query the triple
	result = ka.query_one_solution("lpn:loves(lpn:hello, lpn:world)")
	assert result is not None
	assert result.tokenType() == TokenType.ANSWER_TOKEN
	assert result.isPositive()
	logWarn(str(result))
