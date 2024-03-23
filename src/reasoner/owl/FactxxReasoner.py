from knowrob import *
from pyfactxx import coras
from rdflib import BNode, URIRef, ConjunctiveGraph
from rdflib import Literal as Literal_rdflib
from rdflib.query import ResultRow
from rdflib.namespace import XSD
from typing import List, Tuple, Union

import traceback

class FactxxBackend(DataBackend):
	def __init__(self):
		super(FactxxBackend, self).__init__()
		logDebug("FactxxBackend init.")
		self.is_parse_needed = False
		self.is_update_needed = False
		self.crs = coras.Coras()
		self.bnode_map = {}

	def edb(self) -> ConjunctiveGraph:
		return self.crs._graph

	def parse(self):
		self.is_parse_needed = False
		self.crs.parse()

	def get_blank_node(self, iri: str):
		if iri not in self.bnode_map:
			self.bnode_map[iri] = BNode(iri)
		return self.bnode_map[iri]

	def triple_to_python(self, triple: FramedTriple) -> Tuple[Union[BNode,URIRef], URIRef, Union[URIRef,Literal_rdflib,BNode]]:
		p_uri = URIRef(triple.predicate())
		if triple.isSubjectIRI():
			s_uri = URIRef(triple.subject())
		else:
			s_uri = self.get_blank_node(triple.subject())
		if triple.isObjectIRI():
			o_uri = URIRef(triple.valueAsString())
		elif triple.isObjectBlank():
			o_uri = self.get_blank_node(triple.valueAsString())
		elif triple.xsdType is XSDType.STRING:
			o_uri = Literal_rdflib(triple.valueAsString())
		else:
			o_uri = Literal_rdflib(triple.createStringValue(), datatype=triple.xsdTypeIRI())
		return s_uri, p_uri, o_uri

	def initializeBackend(self, config: PropertyTree) -> bool:
		return True

	def insertOne(self, triple: FramedTriple) -> bool:
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
			all_true = self.insertOne(triple.get()) and all_true
		self.is_update_needed = True
		self.is_parse_needed = True
		return all_true

	def removeOne(self, triple: FramedTriple) -> bool:
		self.edb().remove(self.triple_to_python(triple))
		self.is_update_needed = True
		self.is_parse_needed = True
		return True

	def removeAll(self, triples: TripleContainer) -> bool:
		all_true = True
		for triple in triples:
			all_true = self.removeOne(triple.get()) and all_true
		self.is_update_needed = True
		self.is_parse_needed = True
		return all_true

	def removeAllWithOrigin(self, origin: str) -> bool:
		logWarn("removeAllWithOrigin not implemented for FactxxReasoner")
		return False

## @brief A reasoner that uses the pyfactxx library to perform reasoning on an OWL knowledge base.
# A reasoner that uses the pyfactxx library to perform reasoning on an OWL knowledge base.
# Currently, only the initial loading of OWL files is supported which are then classified and realised
# by the reasoner. All inferred triples are added to the knowledge base.
#
class FactxxReasoner(DataDrivenReasoner):
	def __init__(self):
		super(FactxxReasoner, self).__init__()
		logDebug("FactxxReasoner init.")
		# add all inferred triples to the knowledge base?
		self.synch_to_kb = True
		# skip inferred triples with blank nodes?
		self.ignore_bnodes = True
		self.storage = None

	def initializeReasoner(self, ptree: PropertyTree) -> bool:
		return True

	def setDataBackend(self, storage: DataBackend):
		# ensure that the storage is a FactxxBackend
		if not isinstance(storage, FactxxBackend):
			raise RuntimeError("FactxxReasoner requires a FactxxBackend")
		logDebug("FactxxBackend storage has been initialized.")
		self.storage = storage

	def factxx(self):
		return self.storage.crs.reasoner

	def is_consistent(self) -> bool:
		return self.factxx().is_consistent()

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
		# TODO: there are a lot of useless assertions in the inferred triples.
		#       actually all RDFS inferences could be ignored here in general.
		#       best to query the knowrob vocabulary if type/subclass relation is known already.
		#       for now we skip some particular frequent cases:
		# - `_ rdfs:subClassOf [owl:Thing,owl:NamedIndividual]`
		# - `_ rdf:type [owl:Thing,owl:NamedIndividual,rdf:Property]`
		uri_sub_class_of = URIRef('http://www.w3.org/2000/01/rdf-schema#subClassOf')
		uri_type = URIRef('http://www.w3.org/1999/02/22-rdf-syntax-ns#type')
		uri_thing = URIRef('http://www.w3.org/2002/07/owl#Thing')
		uri_named_individual = URIRef('http://www.w3.org/2002/07/owl#NamedIndividual')
		uri_property = URIRef('http://www.w3.org/1999/02/22-rdf-syntax-ns#Property')
		if row[1] == uri_sub_class_of and row[2] in [uri_thing, uri_named_individual]:
			return False
		if row[1] == uri_type and row[2] in [uri_thing, uri_named_individual, uri_property]:
			return False
		return True

	@staticmethod
	def triple_from_python(kb_triple: FramedTriple, row: ResultRow):
		kb_triple.subject = str(row[0])
		kb_triple.predicate = str(row[1])

		if type(row[2]) is Literal_rdflib:
			literal: Literal_rdflib = row[2]
			kb_triple.setXSDValue(str(literal), str(literal.datatype))
		elif type(row[2]) is URIRef:
			kb_triple.setObjectIRI(str(row[2]))
		elif type(row[2]) is BNode:
			kb_triple.setObjectBlank(str(row[2]))

	def update_triples(self):
		# retrieve all inferred triples
		# NOTE: this also includes the triples that were already inferred before.
		query = 'SELECT ?a ?b ?c WHERE {?a ?b ?c}'
		result_rows: List[ResultRow] = self.storage.crs.query(query, scope='inferred')

		# create a vector of KB triples at once
		filtered_rows = list(filter(self.include_triple, result_rows))
		if len(filtered_rows) == 0:
			logDebug("pyfactxx has no inferences.")
			return
		kb_triples = self.createTriples(len(filtered_rows))

		# convert result to knowrob triples
		triple_index = 0
		for kb_triple, row in zip(kb_triples, filtered_rows):
			self.triple_from_python(kb_triple, row)
			triple_index += 1

		logDebug("pyfactxx inferred " + str(triple_index) + " triples (without bnodes).")
		# set the inferred triples in the knowledge base
		self.setInferredTriples(kb_triples)

	def update(self):
		if not self.storage.is_update_needed:
			return
		self.storage.is_update_needed = False

		reasoner = self.factxx()

		if self.storage.is_parse_needed:
			try:
				self.storage.parse()
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
