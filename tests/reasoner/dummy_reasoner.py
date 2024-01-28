from knowrob import *

class DummyReasoner(ReasonerWithBackend):
	def __init__(self):
		super(DummyReasoner, self).__init__()

	def loadConfig(self, config):
		print("loadConfig: " + str(config))
		return True

	def setDataBackend(self, backend):
		pass

	def getTruthMode(self):
		return TruthMode.CLOSED_WORLD

	def start(self):
		pass

	def stop(self):
		pass

	def insertOne(self, data):
		print("insertOne: " + str(data))
		return True

	def insertAll(self, data):
		return True

	def removeAll(self, literals):
		pass

	def removeOne(self, literal):
		pass

	def getDescription(self, indicator):
		print("getDescription: " + str(indicator))
		return None

	def submitQuery(self, query, ctx):
		return None
