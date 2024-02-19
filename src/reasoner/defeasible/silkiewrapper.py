from knowrob import *
from silkie import *
from rdflib import BNode, URIRef
from rdflib.query import ResultRow
from rdflib.namespace import XSD
from typing import List



class SilkieReasoner(ReasonerWithBackend):
    """
    A reasoner that implements defeasible logic.
    All inferred triples are added to the knowledge base.
    """

    def __init__(self):
        super(SilkieReasoner, self).__init__()

        # add all inferred triples to the knowledge base?
        self.synch_to_kb = True
        # skip inferred triples with blank nodes? TODO: what does this do?
        self.ignore_bnodes = True

        # No need for a "reasoner" object; reasoning is performed via functions
        self.is_parse_needed = False
        self.is_update_needed = False
        self.conclusions = None

        self.rules = TheoryTemplate()
        self._labels = set()
        self.facts = {}
        # add data handler for DFL files
        self.addDataHandler('dfl', lambda ds: self.load_dfl(ds, 'dfl'))
    
    def _newLabel(self):
        retq = len(self._labels)
        k = 0
        while retq in self._labels:
            retq += 2*k + 1
            k += 1
        return retq

    def loadConfig(self, config: ReasonerConfiguration) -> bool:
        # TODO.
        # This reasoner has semantic flags to configure: team/individual defeat, ambiguity blocking/propagation, well-foundedness
        # Will for now assume a default configuration: team defeat, ambiguity blocking, no well-foundedness.
        return True

    def load_dfl_facts(self, data_source: DataSource, file_format) -> bool:
        # TODO: assumes a local file
        logInfo("silkie loading facts file {}.".format(data_source.uri()))
        self.facts = loadDFLFacts(data_source.uri()[len("file://"):], facts=self.facts)
        logDebug("silkie file loaded.")
        # Parsing is done immediately.
        #self.is_parse_needed = False
        self.is_update_needed = True
        return True

    def load_dfl_rules(self, data_source: DataSource, file_format) -> bool:
        # TODO: assumes a local file
        logInfo("silkie loading rules file {}.".format(data_source.uri()))
        self.rules = loadDFLRules(data_source.uri()[len("file://"):], rules=self.rules)
        self._labels = [x._id for x in self.rules.rules.values()]
        logDebug("silkie file loaded.")
        # Parsing is done immediately.
        #self.is_parse_needed = False
        self.is_update_needed = True
        return True

    def is_consistent(self) -> bool:
        if self.is_update_needed:
            self._reason()
        return 0 == len(self.conclusions.contradictions)

    def is_instance_of(self, individual: str, concept: str) -> bool:
        if concept not in self.facts:
            self.facts[concept] = PFact(concept)
        self.facts[concept].addFact(individual, '', STRICT)
        return True

    def create_cls(self, cls_name):
        # note that silkie does not need to declare classes. Rather, it treats a class as a [monadic] predicate, much like SWRL does.
        if cls_name not in self.facts:
            self.facts[cls_name] = PFact(cls_name)
        return self.facts[cls_name] # or name? what should the type be?

    def create_object_role(self, name, role_domain, role_range):
        if name not in self.facts:
            self.facts[name] = PFact(concept)
        # TODO: specify defeasible domain/range?
        dl = self._newLabel()
        rl = self._newLabel()
        domainRule = RuleTemplate(dl, [(name, Variable("x"), Variable("y"))], STRICT, (role_domain, Variable("x"), ""))
        rangeRule = RuleTemplate(rl, [(name, Variable("x"), Variable("y"))], STRICT, (role_range, Variable("y"), ""))
        self.rules.addRules([domainRule, rangeRule])
        self._labels.add(dl)
        self._labels.add(rl)
        return self.facts[name] # or name? what should the type be?

    def get_role_fillers(self, role: str, individual: str) -> List[str]:
        if role not in self.facts:
            return []
        return self.facts[role].getSFacts(individual)

    def parse(self):
        self.is_parse_needed = False

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

    def update_triples(self):
        # create a vector of KB triples at once
        kb_triples = list(self.createTriples(len(self.conclusions.defeasiblyProvable)))

        # convert result to knowrob triples
        for conc, kb_triple in zip(self.conclusions.defeasiblyProvable, kb_triples):
            kb_triple.subject = conc[1]
            kb_triple.predicate = conc[0]
            kb_triple.object = conc[2]
            # Silkie result triples are such that anything appearing in the o position can validly appear in the s position in another triple as well.
            # I.e., there are no "data properties" where the object is some literal which cannot be subject somewhere else.
            kb_triple.objectType = RDFType.RESOURCE

        # set the inferred triples in the knowledge base
        self.setInferredTriples(kb_triples)

    def update(self):
        if not self.is_update_needed:
            return

        theory, _, i2s, _ = buildTheory(self.rules,self.facts,{},debugTheory=False)
        self.conclusions = dflInference(theory,i2s=i2s)
        self.conclusions = idx2strConclusions(conclusions, i2s)
        self.is_update_needed = False

        logDebug("silkie inference start.")
        logDebug("silkie inference finished.")

        if self.is_consistent():
            if self.synch_to_kb:
                logDebug("silkie synch to KB.")
                self.update_triples()
                logDebug("silkie synch to KB done.")
        else:
            logWarn("silkie: inconsistent knowledge base")

    def start(self):
        # start a thread for the reasoner where realise is called.
        self.pushWork(self.update)

    def stop(self):
        pass

    def insertOne(self, triple: StatementData) -> bool:
        # NOTE: currently only the initial loading of DFL files is supported
        logWarn("insertOne not supported")
        return False

    def insertAll(self, triples: List[StatementData]) -> bool:
        # NOTE: currently only the initial loading of DFL files is supported
        logWarn("insertAll not supported")
        return False

    def removeOne(self, triple: StatementData) -> bool:
        # NOTE: currently only the initial loading of DFL files is supported
        return False

    def removeAll(self, triples: List[StatementData]) -> bool:
        # NOTE: currently only the initial loading of DFL files is supported
        return False

    def removeMatching(self, query: RDFLiteral, do_match_many: bool) -> int:
        # NOTE: currently only the initial loading of DFL files is supported
        return 0

    def getDescription(self, indicator: PredicateIndicator) -> PredicateDescription:
        # NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
        return None

    def submitQuery(self, query: RDFLiteral, ctx: QueryContext) -> TokenBuffer:
        # NOTE: inferred triples are added to the knowledge base, so no need to respond to queries
        return None

