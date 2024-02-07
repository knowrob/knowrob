from knowrob import *
from pyfactxx import coras
from rdflib import BNode, URIRef
from rdflib.query import ResultRow
from rdflib.namespace import XSD


class FactxxReasoner(ReasonerWithBackend):
	"""
	A reasoner that uses the pyfactxx library to perform reasoning on an OWL knowledge base.
	Currently, only the initial loading of OWL files is supported which are then classified and realised
	by the reasoner. All inferred triples are added to the knowledge base.
	"""

	def __init__(self):
		super(FactxxReasoner, self).__init__()

		# add all inferred triples to the knowledge base?
		self.synch_to_kb = True
		# skip inferred triples with blank nodes?
		self.ignore_bnodes = True

		self.crs = coras.Coras()
		self.is_parse_needed = False
		self.is_update_needed = False

		# add data handler for OWL files
		self.addDataHandler('rdf-xml', lambda ds: self.load_owl(ds, 'xml'))
		self.addDataHandler('turtle', lambda ds: self.load_owl(ds, 'turtle'))
		self.addDataHandler('json-ld', lambda ds: self.load_owl(ds, 'json-ld'))
		self.addDataHandler('ntriples', lambda ds: self.load_owl(ds, 'ntriples'))
		self.addDataHandler('trix', lambda ds: self.load_owl(ds, 'trix'))
		self.addDataHandler('n3', lambda ds: self.load_owl(ds, 'n3'))

	def loadConfig(self, config: ReasonerConfiguration) -> bool:
		return True

	def load_owl(self, data_source: DataSource, file_format) -> bool:
		logInfo("pyfactxx loading file {}.".format(data_source.uri()))
		self.crs.load(data_source.uri(), format=file_format)
		logDebug("pyfactxx file loaded.")
		self.is_parse_needed = True
		self.is_update_needed = True
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

	def get_role_fillers(self, role: str, individual: str) -> list[str]:
		role = self.factxx().object_role(role)
		return self.factxx().get_role_fillers(individual, role)

	def parse(self):
		self.is_parse_needed = False
		self.crs.parse()

	def include_triple(self, row: ResultRow):
		if self.ignore_bnodes:
			if type(row[0]) is not URIRef:
				# require subject to be a URI
				return False
			if type(row[1]) is not URIRef:
				# require property to be a URI
				return False
		if type(row[2]) is BNode:
			# require object to be a URI or Literal
			return False
		return True

	@staticmethod
	def read_triple(kb_triple: StatementData, row: ResultRow):
		kb_triple.subject = str(row[0])
		kb_triple.predicate = str(row[1])

		if type(row[2]) is Literal:
			literal: Literal = row[2]
			datatype = literal.datatype
			if datatype == XSD.string:
				kb_triple.object = str(row[2])
				kb_triple.objectType = RDFType.STRING_LITERAL
			elif datatype == XSD.integer or datatype == XSD.long:
				kb_triple.objectInteger = int(row[2])
				kb_triple.objectType = RDFType.INT64_LITERAL
			elif datatype == XSD.boolean:
				kb_triple.objectInteger = int(row[2])
				kb_triple.objectType = RDFType.BOOLEAN_LITERAL
			elif datatype == XSD.float or datatype == XSD.float:
				kb_triple.objectDouble = float(row[2])
				kb_triple.objectType = RDFType.DOUBLE_LITERAL
			else:
				logWarn("unknown datatype in inferred triple: " + str(datatype))
		elif type(row[2]) is URIRef:
			kb_triple.object = str(row[2])
			kb_triple.objectType = RDFType.RESOURCE

	def update_triples(self):
		# retrieve all inferred triples
		# NOTE: this also includes the triples that were already inferred before.
		query = 'SELECT ?a ?b ?c WHERE {?a ?b ?c}'
		result_rows: list[ResultRow] = self.crs.query(query, scope='inferred')

		# create a vector of KB triples at once
		filtered_rows = list(filter(self.include_triple, result_rows))
		kb_triples = list(self.createTriples(len(filtered_rows)))

		# convert result to knowrob triples
		triple_index = 0
		for row in filtered_rows:
			kb_triple = kb_triples[triple_index]
			self.read_triple(kb_triple, row)
			triple_index += 1

		# set the inferred triples in the knowledge base
		self.setInferredTriples(kb_triples)

	def update(self):
		if not self.is_update_needed:
			return
		self.is_update_needed = False

		reasoner = self.factxx()

		if self.is_parse_needed:
			self.parse()

		logDebug("pyfactxx realise start.")
		reasoner.classify()
		reasoner.realise()
		logDebug("pyfactxx realise finished.")

		if self.is_consistent():
			if self.synch_to_kb:
				logDebug("pyfactxx synch to KB.")
				self.update_triples()
				logDebug("pyfactxx synch to KB done.")
		else:
			logWarn("pyfactxx: inconsistent knowledge base")

	def start(self):
		# start a thread for the reasoner where realise is called.
		self.pushWork(self.update)

	def stop(self):
		pass

	def insertOne(self, triple: StatementData) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		logWarn("insertOne not supported")
		return False

	def insertAll(self, triples: list[StatementData]) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		logWarn("insertAll not supported")
		return False

	def removeOne(self, triple: StatementData) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		return False

	def removeAll(self, triples: list[StatementData]) -> bool:
		# NOTE: currently only the initial loading of OWL files is supported
		return False

	def removeMatching(self, query: RDFLiteral, do_match_many: bool) -> int:
		# NOTE: currently only the initial loading of OWL files is supported
		return 0

	def getDescription(self, indicator: PredicateIndicator) -> PredicateDescription:
		# NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
		return None

	def submitQuery(self, query: RDFLiteral, ctx: QueryContext) -> TokenBuffer:
		# NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
		return None
