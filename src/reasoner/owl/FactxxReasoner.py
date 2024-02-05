from knowrob import *
from pyfactxx import coras
from rdflib import BNode, URIRef
from rdflib.query import ResultRow
from rdflib.namespace import XSD


class FactxxReasoner(ReasonerWithBackend):
	def __init__(self):
		super(FactxxReasoner, self).__init__()
		self.crs = coras.Coras()
		self.is_parse_needed = False
		self.ignore_bnodes = True
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

	def parse(self):
		self.is_parse_needed = False
		self.crs.parse()

	def update_inferred_triples(self):
		# retrieve all inferred triples
		# NOTE: this also includes the triples that were already inferred before.
		query = 'SELECT ?a ?b ?c WHERE {?a ?b ?c}'
		result_rows: list[ResultRow] = self.crs.query(query, scope='inferred')
		# convert rows to knowrob terms
		knowrob_terms = []
		for row in result_rows:
			if self.ignore_bnodes:
				if type(row[0]) is not URIRef:
					# require subject to be a URI
					continue
				if type(row[1]) is not URIRef:
					# require property to be a URI
					continue
			if type(row[2]) is BNode:
				# require object to be a URI or Literal
				continue
			# TODO: this might do a lot of unneeded copies of the same string! can the memory by mapped maybe?
			#       could add a StringViewTerm or so that is a view on a string.
			#       else maybe provide a factory in knowrob to get concept names etc as StringTerm.
			kb_term = StatementData()
			kb_term.subject = str(row[0])
			kb_term.predicate = str(row[1])

			if type(row[2]) is Literal:
				literal: Literal = row[2]
				datatype = literal.datatype
				if datatype == XSD.string:
					kb_term.object = str(row[2])
					kb_term.objectType = RDFType.STRING_LITERAL
				elif datatype == XSD.integer or datatype == XSD.long:
					kb_term.objectInteger = int(row[2])
					kb_term.objectType = RDFType.INT64_LITERAL
				elif datatype == XSD.boolean:
					kb_term.objectInteger = int(row[2])
					kb_term.objectType = RDFType.BOOLEAN_LITERAL
				elif datatype == XSD.float or datatype == XSD.float:
					kb_term.objectDouble = float(row[2])
					kb_term.objectType = RDFType.DOUBLE_LITERAL
				else:
					logWarn("unknown datatype in inferred triple: " + str(datatype))
					continue
			elif type(row[2]) is URIRef:
				kb_term.object = str(row[2])
				kb_term.objectType = RDFType.RESOURCE
			else:
				# should not happen
				continue

			knowrob_terms.append(kb_term)

		logWarn("FactxxReasoner set inferred")
		self.setInferredTriples(knowrob_terms)

	def update(self):
		reasoner = self.factxx()

		if self.is_parse_needed:
			self.parse()

		reasoner.classify()
		reasoner.realise()

		self.update_inferred_triples()

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
