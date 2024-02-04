from knowrob import *
from pyfactxx import coras
from rdflib import BNode, URIRef
from rdflib.query import ResultRow


class FactxxReasoner(ReasonerWithBackend):
	def __init__(self):
		super(FactxxReasoner, self).__init__()
		self.crs = coras.Coras()
		self.is_parse_needed = False
		# add data handler for OWL files
		self.addDataHandler('rdf-xml', lambda ds: self.load_owl(ds, 'xml'))
		self.addDataHandler('turtle', lambda ds: self.load_owl(ds, 'turtle'))
		self.addDataHandler('json-ld', lambda ds: self.load_owl(ds, 'json-ld'))
		self.addDataHandler('ntriples', lambda ds: self.load_owl(ds, 'ntriples'))
		self.addDataHandler('trix', lambda ds: self.load_owl(ds, 'trix'))
		self.addDataHandler('n3', lambda ds: self.load_owl(ds, 'n3'))

	def load_owl(self, data_source: DataSource, file_format) -> bool:
		logWarn('loading owl file "{}" with format "{}" into FactxxReasoner.'.format(data_source.uri(), file_format))
		self.crs.load(data_source.uri(), format=file_format)
		self.is_parse_needed = True
		return True

	def factxx(self):
		return self.crs.reasoner

	def is_consistent(self) -> bool:
		return self.factxx().is_consistent()

	def is_instance_of(self, individual, concept) -> bool:
		reasoner = self.factxx()
		individual_v = reasoner.individual(individual)
		concept_v = reasoner.concept(individual)
		return self.factxx().instance_of(individual_v, concept_v)

	def create_cls(self, cls_name):
		reasoner = self.factxx()
		cls = reasoner.concept(cls_name)
		return cls

	def create_object_role(self, name, role_domain, role_range):
		reasoner = self.factxx()
		r = reasoner.object_role(name)
		reasoner.set_o_domain(r, role_domain)
		reasoner.set_o_range(r, role_range)
		return r

	def get_unsatisfiable_classes(self) -> list[str]:
		nothing = self.crs.query(
			'select ?n where {	?n <http://www.w3.org/2000/01/rdf-schema#subClassOf> <http://www.w3.org/2002/07/owl#Nothing> . } order by ?p',
			scope='both')

		reasoner = self.factxx()
		reasoner.concept_bottom()

		output = []
		for row in nothing:
			output.append(row[0])
		return output

	def get_role_fillers(self, role: str, individual: str) -> list[str]:
		role = self.factxx().object_role(role)
		# TODO
		values = self.factxx().get_role_fillers(individual, role)
		for v in values:
			print(v.name)

	# return (((s, None, o), context) for o in values)

	def loadConfig(self, config: ReasonerConfiguration) -> bool:
		return True

	def getTruthMode(self) -> TruthMode:
		return TruthMode.OPEN_WORLD

	def parse(self):
		self.is_parse_needed = False
		self.crs.parse()

	def updateInferences(self):
		reasoner = self.factxx()
		# select all triples
		query = 'SELECT ?a ?b ?c WHERE {?a ?b ?c}'
		# retrieve all inferred triples
		# NOTE: this also includes the triples that were already inferred before.
		result_rows: list[ResultRow] = self.crs.query(query, scope='inferred')
		inferred_data = []

		# TODO: first idea: maintain a map of inferences.
		#   Key can be a hash of the triple, maybe via StatementData type as this is used
		#   in KnowRob for assert.
		#   (1) build a new map of inferences
		#   (2) find all keys that appear in the old map but not in the new map.
		#       then tell the KB these triples should be removed.
		#       TODO: side-quest: KB should maintain who asserts triples, but could be ignored for the moment
		#   (3) find all keys that appear in the new map but not in the old map.
		#       then tell the KB these triples should be added.
		#   (4) update the old map with the new map.
		#   TODO: side-quest: how to handle insert into KB? need to have an additional interface.
		#                     also make sure insert below is not called for reasoner that does an assertion.
		#   TODO: side-quest: inferred triples should be marked as such, graph="inferred" ok?

		# TODO: how to obtain only the newly inferred triples?
		# TODO: can bnodes be excluded via the query?
		for row in result_rows:
			if type(row[0]) is not URIRef:
				# require subject to be a URI
				continue
			if type(row[1]) is not URIRef:
				# require property to be a URI
				continue
			if type(row[2]) is BNode:
				# require property to be a URI
				continue
			a_uri = StringTerm(row[0])
			b_uri = StringTerm(row[1])

			# TODO: how are data values handled?
			# TODO
			# inferred_data.append(StatementData(a_uri, b_uri, c_node))
			logWarn("{} {} {}".format(a_uri, b_uri, str(row[2])))

	def update(self):
		reasoner = self.factxx()

		if self.is_parse_needed:
			self.parse()

		reasoner.classify()
		reasoner.realise()

		self.updateInferences()

	def start(self):
		logWarn("start FactxxReasoner")
		if self.is_parse_needed:
			self.parse()
		logWarn("FactxxReasoner parsing done")
		# TODO: start a thread for the reasoner where realise is called. After being done, the thread falls
		# asleep until knowledge is changed. Then it is woken up and realises again.
		self.update()

	def stop(self):
		pass

	def insertOne(self, groundTerm: StatementData) -> bool:
		# FIXME: how to handle the graph selector here?
		#  probably best to configure the reasoner such that only a fitting selector is passed in here
		print("insertOne: " + str(groundTerm))
		return True

	def insertAll(self, groundTerms: list[StatementData]) -> bool:
		all_succeeded = True
		for groundTerm in groundTerms:
			next_succeeded = self.insertOne(groundTerm)
			all_succeeded = all_succeeded and next_succeeded
		return True

	def removeAll(self, literals: list[RDFLiteral]):
		pass

	def removeOne(self, literal: RDFLiteral):
		pass

	def getDescription(self, indicator: PredicateIndicator) -> PredicateDescription:
		print("getDescription: " + str(indicator))
		return None

	def submitQuery(self, query: RDFLiteral, ctx: QueryContext) -> TokenBuffer:
		return None
