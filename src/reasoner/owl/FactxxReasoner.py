from knowrob import *
from pyfactxx import coras
from rdflib import BNode, URIRef, ConjunctiveGraph
from rdflib import Literal as Literal_rdflib
from rdflib.query import ResultRow
from rdflib.namespace import XSD
from typing import List, Tuple, Union

import traceback

class FactxxReasoner(ReasonerWithBackend):
	"""
	A reasoner that uses the pyfactxx library to perform reasoning on an OWL knowledge base.
	Currently, only the initial loading of OWL files is supported which are then classified and realised
	by the reasoner. All inferred triples are added to the knowledge base.
	"""

	def __init__(self):
		super(FactxxReasoner, self).__init__()
		logDebug("FactxxReasoner init.")

		# add all inferred triples to the knowledge base?
		self.synch_to_kb = True
		# skip inferred triples with blank nodes?
		self.ignore_bnodes = True
		self.bnode_map = {}

		self.crs = coras.Coras()
		self.is_parse_needed = False
		self.is_update_needed = False

	def loadConfig(self, config: ReasonerConfiguration) -> bool:
		return True

	def load_owl(self, uri: str, file_format: str) -> bool:
		logInfo("pyfactxx loading file {}.".format(uri))
		self.crs.load(uri, format=file_format)
		logDebug("pyfactxx file loaded.")
		self.is_parse_needed = True
		self.is_update_needed = True
		return True

	def factxx(self):
		return self.crs.reasoner

	def edb(self) -> ConjunctiveGraph:
		return self.crs._graph

	def is_consistent(self) -> bool:
		return self.factxx().is_consistent()

	def is_instance_of(self, individual, concept) -> bool:
		reasoner = self.factxx()
		individual_v = reasoner.individual(individual)
		concept_v = reasoner.concept(concept)
		return self.factxx().instance_of(individual_v, concept_v)

	def get_role_fillers(self, individual: str, role: str) -> List[str]:
		reasoner = self.factxx()
		individual_v = reasoner.individual(individual)
		role_v = reasoner.object_role(role)
		return reasoner.get_role_fillers(individual, role_v)

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

	def get_blank_node(self, iri: str):
		if iri not in self.bnode_map:
			self.bnode_map[iri] = BNode(iri)
		return self.bnode_map[iri]

	def resource_to_python(self, iri: str):
		if iri.startswith("_"):
			return self.get_blank_node(iri)
		elif iri.startswith("genid"):
			# FIXME: where are these genid's coming from?
			return self.get_blank_node(iri)
		else:
			return URIRef(iri)

	def triple_to_python(self, triple: StatementData) -> Tuple[Union[BNode,URIRef], URIRef, Union[URIRef,Literal_rdflib,BNode]]:
		s_uri = self.resource_to_python(triple.subject)
		p_uri = URIRef(triple.predicate)
		if triple.objectType == RDFType.RESOURCE:
			return s_uri, p_uri, self.resource_to_python(triple.object)
		elif triple.objectType == RDFType.INT64_LITERAL:
			return s_uri, p_uri, Literal_rdflib(triple.objectInteger)
		elif triple.objectType == RDFType.DOUBLE_LITERAL:
			return s_uri, p_uri, Literal_rdflib(triple.objectDouble)
		elif triple.objectType == RDFType.BOOLEAN_LITERAL:
			return s_uri, p_uri, Literal_rdflib(triple.objectInteger == 1)
		else:
			return s_uri, p_uri, Literal_rdflib(triple.object)

	@staticmethod
	def triple_from_python(kb_triple: StatementData, row: ResultRow):
		kb_triple.subject = str(row[0])
		kb_triple.predicate = str(row[1])

		if type(row[2]) is Literal_rdflib:
			literal: Literal_rdflib = row[2]
			datatype = literal.datatype
			kb_triple.object = str(row[2])
			if datatype == XSD.string:
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
		elif type(row[2]) is BNode:
			bnode_name = str(row[2])
			if bnode_name.startswith("_"):
				kb_triple.object = bnode_name
			else:
				kb_triple.object = "_" + bnode_name
			kb_triple.objectType = RDFType.RESOURCE

	def update_triples(self):
		# retrieve all inferred triples
		# NOTE: this also includes the triples that were already inferred before.
		query = 'SELECT ?a ?b ?c WHERE {?a ?b ?c}'
		result_rows: List[ResultRow] = self.crs.query(query, scope='inferred')

		# create a vector of KB triples at once
		filtered_rows = list(filter(self.include_triple, result_rows))
		if len(filtered_rows) == 0:
			logDebug("pyfactxx has no inferences.")
			return
		kb_triples = list(self.createTriples(len(filtered_rows)))

		# convert result to knowrob triples
		triple_index = 0
		for row in filtered_rows:
			kb_triple = kb_triples[triple_index]
			self.triple_from_python(kb_triple, row)
			triple_index += 1

		logDebug("pyfactxx inferred " + str(triple_index) + " triples (without bnodes).")
		# set the inferred triples in the knowledge base
		self.setInferredTriples(kb_triples)

	def update(self):
		if not self.is_update_needed:
			return
		self.is_update_needed = False

		reasoner = self.factxx()

		if self.is_parse_needed:
			# FIXME: for some reason, the parse method is not working as expected.
			#        an assertion fails in the parser of pyfactxx, it seems it cannot retrieve the
			#        owl:onProperty property of an owl:Restriction. Not sure why this happens.
			#        with load_owl it worked fine though, at least for the SOMA ontology alone
			#        not sure if in this case DUL and the other files were loaded as well.
			# FIXME: also the exception is not caught in C++ and terminates the program.
			#        probably related to special handling for failed assertions in the parser.
			try:
				self.parse()
			except Exception as e:
				logError("pyfactxx parse failed: " + str(e))
				traceback.print_exc()
				return

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
		#self.update()

	def stop(self):
		pass

	def insertOne(self, triple: StatementData) -> bool:
		py_triple = self.triple_to_python(triple)
		self.edb().add(py_triple)
		self.is_update_needed = True
		self.is_parse_needed = True
		return True

	def insertAll(self, triples: TripleContainer) -> bool:
		# TODO: use addN instead of add
		# self.edb().addN(FactxxReasoner.triples_to_python(triples))
		all_true = True
		for triple in triples:
			all_true = self.insertOne(triple) and all_true
		self.is_update_needed = True
		self.is_parse_needed = True
		return all_true

	def removeOne(self, triple: StatementData) -> bool:
		self.edb().remove(self.triple_to_python(triple))
		self.is_update_needed = True
		self.is_parse_needed = True
		return True

	def removeAll(self, triples: TripleContainer) -> bool:
		# TODO: there is no trivial interface to remove all triples from the graph at once.
		#       however there seems to be a db transaction interface. e.g.:
		#       self.edb().commit() is there, and rollbacks are supported as well.
		#       so it should be possible to implement removeAll by using a transaction.

		all_true = True
		for triple in triples:
			all_true = self.removeOne(triple) and all_true
		self.is_update_needed = True
		self.is_parse_needed = True
		return all_true

	def removeAllWithOrigin(self, origin: str) -> bool:
		logWarn("removeAllWithOrigin not implemented for FactxxReasoner")
		return False

	def removeMatching(self, query: RDFLiteral, do_match_many: bool) -> int:
		# NOTE: currently only the initial loading of OWL files is supported
		logWarn("removeMatching not implemented for FactxxReasoner")
		return 0

	def getDescription(self, indicator: PredicateIndicator) -> PredicateDescription:
		# NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
		return None

	def submitQuery(self, query: RDFLiteral, ctx: QueryContext) -> TokenBuffer:
		# NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
		logWarn("submitQuery not implemented for FactxxReasoner")
		return None
