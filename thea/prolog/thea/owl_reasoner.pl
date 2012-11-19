:- module(owl_reasoner,
	  [ dig_reasoner_id/2,
	    dig_new_kb/3,
	    dig_release_kb/3,
	    dig_tell/4,
	    dig_ask/4,
	    owl_as2dig/2,

	    dig_kb/2,

	    owl_as2prolog/2
	  ]).


:-use_module(library('http/http_client')).
:-use_module(library('sgml')).
:-use_module(library('sgml_write')).
:-use_module('owl_parser.pl').

:- dynamic(dig_kb/2).


/*
         	OWL DIG Language Wrapper 
	     -------------------------------

This submodule implements a wrapper around a DIG interface. It allows
communication between a Prolog programm and a DIG enable reasoner such
as Racer and Pellet. The wrapper has been tested with Pellet DL
reasoner.

	     -------------------------------

*/

		    
% dig_reasoner_id(+ReasonerURL, ?Response)
%
%        Identification Request from the reasoner in ReasonerURL
%        The Response is the XML response returned by the reasoner.

dig_reasoner_id(ReasonerURL,Response) :-
	dig_request(ReasonerURL, [element(getIdentifier,[],[])] , Response).



% dig_new_kb(+ReasonerURL,+NewKB,?Result) 
%
%        Requests a new knowledge base from the reasoner.
%	 If successfull the URI is stored as a dig_kb(NewKB,URI)
%	 predicate. 
%        If the NewKB already exists in dig_kb then no 
%	 request is made to the reasoner. 
%	 Result is the response from the reasoner.

dig_new_kb(_ReasonerURL, NewKB, Result) :-
	var(NewKB), !, Result = 'NewKB (arg2) must be instantiated'.

dig_new_kb(ReasonerURL,NewKB,Result) :-
	(   nonvar(NewKB), dig_kb(NewKB,_), Result = 'KB already exists', ! 
	; 
	Request = [element(newKB,[],[])],
	    dig_request(ReasonerURL, Request , Result),
	    (   
	    Result = [element(response,_,[element(kb,[uri=KBURI],[])])] , !,
		assert(dig_kb(NewKB,KBURI))
	    ; 
	    true)
	).


% dig_release_kb(+ReasonerURL, +KBName, ?Result) 
%
%	 Requests reasoner to release existing KBName.
%	 If successfull the KB is also removed from the
%	 dig_kb(NewKB,URI) facts.

dig_release_kb(_ReasonerURL, KBName, Result) :-
	var(KBName), !,  Result = 'KBName (arg2) must be instantiated'. 

dig_release_kb(ReasonerURL,KBName, Result) :-
	dig_kb(KBName,KBURI),
        Request = [element(releaseKB,[uri=KBURI],[])],
	dig_request(ReasonerURL, Request , Result),
	Result = [element(response,_,[element(Res2,[],[])])],
	(   Res2 = ok, retract(dig_kb(KBName,KBURI)),! ; true).



% dig_tell(+ReasonerURL,+KBName,+Tells,?Response) 
%
%	 Sends a list of tell requests to the DIG reasoner. 
%        Response is the unparsed XML response from reasoner.
%        KBName must be instantiated and in the dig_kb.

dig_tell(ReasonerURL,KBName,Tells,Response) :- 
	nonvar(KBName),dig_kb(KBName,KB),!,
        Request = [element(tells,[uri=KB],Tells)],
	dig_request(ReasonerURL, Request , Response).
		

% dig_ask(+ReasonerURL,+KBName,+Query,?Result) 
%
%	 Sends a DIG ask Query, expressed in DIG Ask language to the
%	 reasoner. 
%	 The query is transformed to the XML representation by the
%	 dig_ask/2 predicate. 
%	 Reasoner's response is processed by the dig_ask_response/3
%	 and the Result is a List representation of the DIG
%	 response lagnuage.

dig_ask(ReasonerURL,KBName,Query,Result) :- !,
	nonvar(KBName),dig_kb(KBName,KBURI),
	dig_ask(Query,Asks),
	(   dig_request(ReasonerURL, [element(asks,[uri=KBURI],Asks)] , 
			[element(_ResponseElement,_Attrs,[Responses])]),!,
	    (   Responses = element(error,[code=ErrCode,message=ErrMessage],[ErrDescription]), !, 
		Result=error(ErrCode,ErrMessage,ErrDescription);
	    dig_ask_response(Query,Responses,Result)); 
	Result = []).


% dig_request(+ReasonerURL, +Request, ?Response)
%
%	 Lower level predicate. Sends a DIG Request to the reasoner and
%	 get's its Response. It is using SWI Prolog's HTTP and SGML
%	 packages. 

dig_request(Reasoner_URL,Request,Response) :-
	% write Request to predefined file
	Filename = '_thea_dir_request.xml',
	open(Filename, write,St,[encoding(utf8)]), 
	xml_write(St,Request,[layout(false)]),close(St),  % layout false to avoid newlines
	size_file(Filename,RLength),RLength1 is RLength - 2, % size_file is not counting correctly newlines in 
	                                                     % windows environment.
	catch(http_post(Reasoner_URL,file(Filename),Result,[request_header('Content-Type' = 'text/xml'), 
						 request_header('Content-Length' = RLength1)]),_,true),
	% write reasoner response to predefined file
	FilenameResponse = '_thea_dir_response.xml',
	open(FilenameResponse,write,St),
	write(St,Result),
	close(St),
	% Read response from file into an xml structure
	open(FilenameResponse,read,St),
	load_structure(St,Response,[dialect(xml),space(sgml)]),
	close(St).



% dig_ask(+ASKQuery, ?XMLRepresentation)
%
%	 Lower level predicate. It converts a query in the DIG Ask
%	 language to the XML representation required by the reasoner

dig_ask(allConceptNames,[element(allConceptNames,[id=q1],[])]).
dig_ask(allRoleNames,[element(allRoleNames,[id=q1],[])]).
dig_ask(allIndividuals,[element(allIndividuals,[id=q1],[])]).

dig_ask(satisfiable(C),[element(satisfiable,[id=q1],[C1])]) :-
	owl_as2dig(description(C,_),C1).
dig_ask(subsumes(C1,C2),[element(subsumes,[id=q1],[C1D,C2D])]) :-
	owl_as2dig(description(C1,_),C1D),owl_as2dig(description(C2,_),C2D).
dig_ask(disjoint(C1,C2),[element(disjoint,[id=q1],[C1D,C2D])]) :-
	owl_as2dig(description(C1,_),C1D),owl_as2dig(description(C2,_),C2D).

dig_ask(parents(C),[element(parents,[id=q1],[C1])]) :-
	owl_as2dig(description(C,_),C1).
dig_ask(children(C),[element(children,[id=q1],[C1])]) :-
	owl_as2dig(description(C,_),C1).
dig_ask(ancestors(C),[element(ancestors,[id=q1],[C1])]) :-
	owl_as2dig(description(C,_),C1).
dig_ask(descendants(C),[element(descendants,[id=q1],[C1])]) :-
	owl_as2dig(description(C,_),C1).
dig_ask(equivalents(C),[element(equivalents,[id=q1],[C1])]) :-
	owl_as2dig(description(C,_),C1).

dig_ask(instances(C),[element(instances,[id=q1],[C1])]) :- !,
	owl_as2dig(description(C,_),C1).
dig_ask(types(I),[element(types,[id=q1],[element(individual,[name=I],[])])]).
dig_ask(instance(I,C),[element(instance,[id=q1],[element(individual,[name=I],[]),C1])]) :- !,
	owl_as2dig(description(C,_),C1).
dig_ask(roleFillers(I,R),[element(roleFillers,[id=q1],[element(individual,[name=I],[]),
						    element(ratom,[name=R],[])])]) :- !.
dig_ask(relatedIndividuals(R),[element(relatedIndividuals,[id=q1],[element(ratom,[name=R],[])])]) :- !.

dig_ask(toldValues(I,R),[element(toldValues,[id=q1],[element(individual,[name=I],[]),
						    element(attribute,[name=R],[])])]) :- !.




% dig_ask_response(+ASKQuery, +ReasonerResult, ?Result)
%
%	 Lower level predicate. It converts the XML
%	 representation (ReasonerResult) of the responses to an ASK
%	 query to a list representation of the Results based on DIGs
%	 response language.


dig_ask_response(allConceptNames,element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	
dig_ask_response(allRoleNames,element(roleSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(role),Synonyms,Result1),flatten(Result1,Result).		
dig_ask_response(allIndividuals,element(individualSet,_,Individuals),Result) :-!,
	maplist(map_concepts(concept),Individuals,Result).		


dig_ask_response(parents(_),element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	

dig_ask_response(children(_),element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	

dig_ask_response(ancestors(_),element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	

dig_ask_response(descendants(_),element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	

dig_ask_response(equivalents(_),element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	


dig_ask_response(instances(_),element(individualSet,_,Individuals),Result) :-!,
	maplist(map_concepts(concept),Individuals,Result).		

dig_ask_response(types,element(conceptSet,_,Synonyms),Result) :-!,
	maplist(map_synonyms(concept),Synonyms,Result1),flatten(Result1,Result).	

dig_ask_response(instance(_,_),element(Result,_,_),Result) :- !.

dig_ask_response(roleFillers(_,_),element(individualSet,_,Individuals),Result) :-!,
	maplist(map_concepts(concept),Individuals,Result).	

dig_ask_response(relatedIndividuals(_),element(individualPairSet,_,IndividualPairs),Result) :-!,
	maplist(map_individual_pairs(concept),IndividualPairs,Result).	

dig_ask_response(_,Result,Result) :- !.



%
%	 Mapping predicates to convert between XML representation and
%	 list representation of ask responses.
%

map_concepts(_What,element(catom,[name=X],[]),X) :- !.
map_concepts(_What,element(ratom,[name=X],[]),X) :- !.
map_concepts(_What,element(individual,[name=X],[]),X) :- !.
map_concepts(_What,element(top,[],[]),top) :- !.
map_concepts(_What,element(bottom,[],[]),top) :- !.
map_concepts(_What,X,X).

map_synonyms(What,element(synonyms,_,Set1),Set2) :-
	     maplist(map_concepts(What),Set1,Set2).

map_individual_pairs(What,element(individualPair,_,Set1),Set2) :-
	     maplist(map_concepts(What),Set1,Set2).





% owl_as2dig(+OwlAsTerm,?TellElement) 
%
%	 Predicate to convert a Thea prolog OWL abstract term into
%	 a DIG Tell element ready to be submitted to the DIG
%	 reasoner via a tell request.



%
%  Definition of a class (concept).
%

owl_as2dig(class(C,_,complete,_,[intersectionOf([])]),element(defconcept,[name=C],[])) :- !.

owl_as2dig(class(C,_,complete,_,[]),element(defconcept,[name=C],[])) :- !.


%
%  Definition of a class (concept) and 	equivalent class
%

owl_as2dig(class(C,_,complete,_,[D]),[element(defconcept,[name=C],[]),R1,R2]) :- !,
	owl_as2dig(subclassOf(C,D),R1),
	owl_as2dig(subclassOf(D,C),R2).

%
%  A class completely defined as an intersection of descriptions
%

owl_as2dig(class(C,_,complete,_,DL),[element(defconcept,[name=C],[])|R]) :- !,
	owl_as2dig(subclassOf(C,intersectionOf(DL)),R).


%
%  A class partially defined as a subclass of descriptions
%

owl_as2dig(class(C,_,partial,_,DL),[element(defconcept,[name=C],[])|R]) :- !,
	% subclassOf(C,D)
	maplist(map_subclass_dig(C),DL,R).

%
%  Subclass axiom 
%

owl_as2dig(subclassOf(A,B),R) :- !,     
     owl_as2dig(description(A,_),Rb),
     owl_as2dig(description(B,_),Rh),
     R = element(impliesc,[],[Rb,Rh]).

%
%  Intersection description produces an 'and' of tells
%

owl_as2dig(description(intersectionOf(DL),X),element(and,[],R)):- !,
	owl_as2dig(description_list(DL,X,','),R).


%
%  Union description produces an 'or' of tells
%

owl_as2dig(description(unionOf(DL),X),element(or,[],R)):-!,
	owl_as2dig(description_list(DL,X,';'),R).

%
%  Union description -> not tell
%

owl_as2dig(description(complementOf(D),_),R) :- !,
	owl_as2dig(description(D,_),D1),
	R = element(not,[],[D1]).


%
%  'One of' description -> iset tell element.
%

owl_as2dig(description(oneOf(DL),_),R) :- !,
	owl_as2dig(description_list(oneOf(DL),_,_),L),
	R = element(iset,[],L).

%
%  Value restriction on property --> some iset
%

owl_as2dig(description(restriction(PropertyID,value(Value)),_),R) :- !, 
	R = element(some,[],[element(ratom,[name=PropertyID],[]),
			     element(iset,[],[element(individual,[name=Value],[])])]).


%
%  Universal qualifier on property --> all(R, E)
%

owl_as2dig(description(restriction(PropertyID,allValuesFrom(Descr)),_),R) :-  !,
	owl_as2dig(description(Descr,_),D1),
	R =  element(all,[], [element(ratom,[name=PropertyID],[]),
			      D1]).

%
%  Existential qualifier on property --> some(R, E)
%

owl_as2dig(description(restriction(PropertyID,someValuesFrom(Descr)),_),R) :-  !,
	owl_as2dig(description(Descr,_),D1),
	R =  element(some,[], [element(ratom,[name=PropertyID],[]),
			      D1]).

%
%  Cardinalities max, min, both 
%

owl_as2dig(description(restriction(PropertyID,maxCardinality(literal(type(_Type,C)))),_),R) :-  !,
	R = element(atmost,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])]).
owl_as2dig(description(restriction(PropertyID,maxCardinality(C)),_),R) :-  !,
	R = element(atmost,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])]).


owl_as2dig(description(restriction(PropertyID,minCardinality(literal(type(_Type,C)))),_),R) :-  !,
	R = element(atleast,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])]).
owl_as2dig(description(restriction(PropertyID,minCardinality(C)),_),R) :-  !,
	R = element(atleast,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])]).


owl_as2dig(description(restriction(PropertyID,cardinality(literal(type(_Type,C)))),_),R) :-  !,
	R = element(and,[],[element(atleast,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])]),
	     element(atmost,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])])]).

owl_as2dig(description(restriction(PropertyID,cardinality(C)),_),R) :-  !,
	R = element(and,[],[element(atleast,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])]),
	     element(atmost,[num=C],[element(ratom,[name=PropertyID],[]),element(top,[],[])])]).


%
%  Description List --> returns a list of description conversions.
%

owl_as2dig(description_list([],_,_),[]) :-!.

owl_as2dig(description_list([Descr|Rest],X,_),[H|Tail]) :-
	owl_as2dig(description(Descr,X),H),!,	
	owl_as2dig(description_list(Rest,X,_),Tail).

%
%  All remaining descriptions translate to a catom element
%

owl_as2dig(description(Any,_),element(catom, [name=Any],[])).


%
%  Property translation. Translates to a set of subproperties, domain,
%  range and property attribute tells that are handled through a set of
%  mapping functions.
%

owl_as2dig(property(PID,_Deprecated,_AnnotationList,PID_SuperList,PTList,PID_DomainList,PID_RangeList),Tells) :- !,
	maplist(map_subproperty_dig(PID),PID_SuperList,L1),
	maplist(map_domain_dig(PID),PID_DomainList,L2),
	maplist(map_range_dig(PID),PID_RangeList,L3),
	process_pt_dig(PID,PTList,L4),
	flatten([L1,L2,L3,L4],Tells).

%
%  Individual translation. Translates to a set of instancof (type),
%  property and role tells that are handled through approrpiate
%  mapping functions.
%

owl_as2dig(individual(IID,_,TypeList,PropertyList),L) :- !,
       maplist(map_instanceof_dig(IID),TypeList,L1),
       maplist(map_property_dig(IID), PropertyList,L2),
       flatten([element(defindividual,[name=IID],[]),L1,L2], L).


%
%  Mapping of a subclass, calls the subclassOf translation
%

map_subclass_dig(C,D,R) :- 
	owl_as2dig(subclassOf(C,D),R).


%
%  Mapping of a subproperty relation.
%

map_subproperty_dig(PID,SuperP,element(impliesr,[],[element(ratom,[name=PID],[]), 
						    element(ratom,[name=SuperP],[])])).

%
%  Mapping of a domain definition
%

map_domain_dig(PID,Domain,Tell) :-
	owl_as2dig(description(Domain,_),E),
	Tell = element(domain,[],[element(ratom,[name=PID],[]),E]).

%
%  Mapping of a range definition. If a datatype property then either a
%  rangeint or a rangestring. If it is an objectproperty then get the
%  range from the description of the doamin.
%

map_range_dig(PID,Domain,Tell) :-
	(   property(PID,_,_,_,[datatypeProperty,_,_,_,_,_],_,Range),!,
	    (	member('http://www.w3.org/2001/XMLSchema#int',Range),Val = rangeint,! ; Val = rangestring),
	    Tell = element(Val,[],[element(attribute,[name=PID],[])]) ;
	owl_as2dig(description(Domain,_),E),
	Tell = element(range,[],[element(ratom,[name=PID],[]),E])).

%
%  Translate property attributes. Define attribute or role. Create
%  functional, inverse, transitive, symmetric and inverse
%  property tells.
%

process_pt_dig(PID, [Type,F,IF,T,S,iof(Inv)],[Typet, Ft,IFt,Tt,St,INVt]) :-
	(   Type = datatypeProperty, !, Typet = element(defattribute,[name=PID],[]) ; Typet = element(defrole,[name=PID],[])),
	(   nonvar(F), !, Ft = element(functional,[],[element(ratom,[name=PID],[])]) ;  Ft = []),
	(   nonvar(IF), !, IFt = element(inverse,[],[element(ratom,[name=PID],[])])  ; IFt = []),
	(   nonvar(T), !, Tt = element(transitive,[],[element(ratom,[name=PID],[])]) ; Tt = []),
	(   nonvar(S), !, St = element(equalr,[],[element(ratom,[name=PID],[]),
						      element(inverse,[],[element(ratom,[name=PID],[])])]) ; St = []),

	(   nonvar(Inv), !, INVt = element(equalr,[],[element(ratom,[name=PID],[]), 
						      element(inverse,[],[element(ratom,[name=Inv],[])])]) ; INVt = []).

%
%  Mapping of a class membership (instanceof tell).
%

map_instanceof_dig(Instance,Type,Tell) :-
	owl_as2dig(description(Type,_),E),
	Tell = element(instanceof,[],[element(individual,[name=Instance],[]),E]).


%
%  Mapping of an instance property values. For a datatypeproperty
%  value(I, P, V) for an objectproperty related(I, R, I). 
%

map_property_dig(IID, value(P,V), Tells):-	
	(   property(P,_,_,_,[datatypeProperty,_,_,_,_,_],_,Range),! ,
	    (	V = literal(type(_T,V1)),!
	    ; V1 = V 
	    ),
	    (	member('http://www.w3.org/2001/XMLSchema#int',Range),Val = ival,! 
	    ; 
	    Val = sval 
	    ), 
	    Tells = element(value,[],[element(individual,[name=IID],[]),
				      element(attribute,[name=P],[]),
				      element(Val,[],[V1])]) 
	;
	Tells = element(related,[],[element(individual,[name=IID],[]),
				      element(ratom,[name=P],[]),
				      element(individual,[name=V],[])])
	).


 

/*
         	OWL_AS 2 Prolog submodule
	     -------------------------------

This submodule converts an OWL ontology represented as OWL
abstract syntax terms into a Prolog program. The mapping implements the
idea of Description Logic Programs [Grossof]. Similar work has been also
done in the dlpconvert tool.

	     -------------------------------
*/


% owl_as2prolog(+OwlAsTerm,+Options) 
%
%	 Converts the prolog OWL abstract syntax term (as parsed by
%	 OWl parser) into prolog logic code, based on the mapping
%	 proposed by [Grosof] in the context of DLP. The prolog code
%	 is written into the current output stream, so
%	 redirecting the output stream into a file is suggested in order
%	 to capture the generated code. Options are generic options to
%	 modify the behaviour of the code generation. Currently only the
%	 no_base(Namespace) is supported. This option tells the code
%	 generator not to prefix the prolog predicates with the
%	 namespace prefix.



owl_as2prolog(OwlAsTerm,Options) :- 
	owl_as2prolog(OwlAsTerm,R,_),
	owl_write_prolog_code(R,Options).


% owl_write_prolog_code(+Term,+Options) 
%
%	 Term is an intermediate format generated from the
%	 owl_as2prolog/3 predicate. This predicate handles the
%	 prolog code generation from this intermediate format
%	 into prolog code. 
%        For Options see the owl_as2prolog/2 predicate.


%
% Generate code for each item in the list.
% 

owl_write_prolog_code([],_) :- !.


owl_write_prolog_code([H|T],Options) :-
	owl_write_prolog_code(H,Options),!, 
	owl_write_prolog_code(T,Options).


%
% Generate code for the or (;) prolog construct.
% 

owl_write_prolog_code(;(A,B),Options) :- !,
	write('('), owl_write_prolog_code(A,Options), 	write(';'), 
	owl_write_prolog_code(B,Options), write(')').

%
% Generate code for the and (;) prolog construct.
% 

owl_write_prolog_code( (A,B), Options ) :- !,
	owl_write_prolog_code(A,Options), 
	write(','), 
	owl_write_prolog_code(B,Options).


%
% Generate code for a prolog rule Head :- Body 
% 

owl_write_prolog_code(:-(H,B),Options) :-!,
	(   H = false , ! 
	; 
	B = false , ! 
	; 
	H = [], ! 
	; 
	H = [_|_] , !, 
	    maplist(map_head_conjunction(B),H,R), 
	    owl_write_prolog_code(R,Options) % rewrite rule (a,b) :- c ==> a :- c and b:-c

	; 
	H = :-(H1,H2) , !, 
	    owl_write_prolog_code(:-(H1,(H2,B)),Options) % rewrite rule a :- b) :- c ==> a :- b,c
	;  
	B = none, !, % It is a fact (no body).
	    owl_write_prolog_code(H,Options), write('.'), nl
	; 
	owl_write_prolog_code(H,Options), write(':-'), % normal rule H:-B. 
	    nl, write('     '), 
	    owl_write_prolog_code(B,Options), 
	    write('.'), nl
	).

%
% Generate code for a 'class' predicate:  C(X) or C(individual). 
% 

owl_write_prolog_code(class(X,Y),Options) :- !,
	collapse_ns(X,X1,'_',Options),collapse_ns(Y,Y1,':',[]),
	(   var(Y), !, 	
	    writeq(X1), write('(X)') 
	;
	Y = y , !,  
	    writeq(X1), write('('), write('Y'), write(')')
	;
	writeq(X1), write('('), writeq(Y1), write(')')
	).    

%
% Generate code for a 'property' predicate: P(X,Y) or
% P(class,individual) or P(individual, individual).
% 
	
owl_write_prolog_code(property(P,X,Y),Options) :- !, 
	collapse_ns(P,P1,'_',Options),collapse_ns(Y,Y1,':',[]),collapse_ns(X,X1,':',[]),
	writeq(P1),  write('('),
	(   X = x, !, write('X') ; 
	X = y, !, write('Y') ; 
	X = z, !, write('Z') ; 
	X = var , !, write('_'); 
	writeq(X1)
	),	
	write(','),
	(   Y = x, !, write('X') ; 
	Y = y, !, print('Y') ; 
	Y = z, !, print('Z') ; 
	Y = var , !, print('_'); 
	writeq(Y1)
	),
	write(')').

%
% none generates nothing.
%

owl_write_prolog_code(none,_Options) :- !.

%
% otherwise generate the Term itself
%

owl_write_prolog_code(Term,_Options) :-
	writeq(Term).

%
% used in case of conjunction in the head. Used in rewrite rule
%( a,b) :- c ==> a :- c and b:-c 
%

map_head_conjunction(B,H, :-(H,B)).



% owl_as2prolog(+OwlAsTerm, -ResultTerm, ?Mode) 
%
%	 Predicate to convert a Thea prolog OWL abstract term into
%	 the intermediate term used for prolog (logic) code generation.
%	 The Mode is used to differentiate the convertion depending on
%	 wether the OWL construct appears in the head or in a body of a
%	 prolog rule. It cna be on of head, body and fact.
%
%	 The mappings for the class descriptions are summarised in the
%	 following table for each mode.
%   
% Description	  Head                 Body	        Fact
% -----------------------------------------------------------------
% intersectionOf a,b,c, +rewrite rule  a,b,c            -
% unionOf        -                     a;b;c            a. b. c.
% compl          -                     -                -  
% one of                                                -
% restr value    p(ID,V)               p(ID,V)          p(ID,V) 
% restr all      C(Y):-P(X,Y),D(X)     -                C(Y):-P(ID,Y). 
% restr some     -		       C(X):-P(X,Y),D(Y) -



% 
% A class with no description generates none (no code).
%

owl_as2prolog(class(_,_,complete,_,[intersectionOf([])]),none,_) :- !.


% 
% A complete class declaration with a single descrption element is
% equivalent to this description 
%

owl_as2prolog(class(C,_,complete,_,[D]),[R1,R2],_) :- !,
	% equivalent
	owl_as2prolog(subclassOf(C,D),R1,_),
	owl_as2prolog(subclassOf(D,C),R2,_).


% 
% A complete class declaration with multiple descrption elements
% is a subclass of the intersection of the elements
%

owl_as2prolog(class(C,_,complete,_,DL),[R],_) :- !,
	% intersectionOf  
	owl_as2prolog(subclassOf(C,intersectionOf(DL)),R,_).


% 
% A partial class declaration with multiple descrption elements
% is a subclass of each of the elements.
%

owl_as2prolog(class(C,_,partial,_,DL),R,_) :- !,
	% subclassOf(C,D)
	maplist(map_subclass_prolog(C),DL,R).

% 
% Subclass(Class,Superclass) ==> C(X) implies S(X) or S(X) :- C(X).
%

owl_as2prolog(subclassOf(A,B),R,_) :- !,     
     owl_as2prolog(description(A,_),Rb,body),
     owl_as2prolog(description(B,_),Rh,head),
     R = :-(Rh,Rb).

% 
% Intersection of descriptions does not generate anything in fact mode.
%

owl_as2prolog(description(intersectionOf(_),_),false,fact):- !.


% 
% Intersection of descriptions generates a comma separated list of
% descriptions in either head or body modes. 
%

owl_as2prolog(description(intersectionOf(DL),X),R,Param):- !,
	owl_as2prolog(description_list(DL,X,','),R,Param).


% 
% Union (use of Or) cannot be handled in the head of a rule.
%

owl_as2prolog(description(unionOf(_),_),false,head):-!.

% 
% Union generates ; separated terms.
%

owl_as2prolog(description(unionOf(DL),X),R,body):-!,
	owl_as2prolog(description_list(DL,X,';'),R,body).

owl_as2prolog(description(unionOf(DL),X),R,fact):-!,
	owl_as2prolog(description_list(DL,X,';'),R,fact).


%
% Complement of (Not) is not handled in this conversion
%

owl_as2prolog(description(complementOf(_),_),false,_) :- !.


%
% OneOf is handled with membership only in body of rules.
%

owl_as2prolog(description(oneOf(DL),_),member(_,DL),body) :- !.
owl_as2prolog(description(oneOf(_),_),false,_) :- !.


%
% Value property description generates a property term (predicate)
%

owl_as2prolog(description(restriction(PropertyID,value(Value)),X),R,_) :- 
	R = property(PropertyID,X,Value),!.

%
% Universal property description. See table above
%

owl_as2prolog(description(restriction(_,allValuesFrom(_)),_),false,body) :-  !.

owl_as2prolog(description(restriction(PropertyID,allValuesFrom(Descr)),_),R,head) :-  !,
	owl_as2prolog(description(Descr,y),D,head),
	R =  :-(D,property(PropertyID,x,y)).

owl_as2prolog(description(restriction(PropertyID,allValuesFrom(Descr)),ID),R,fact) :-  !,
	owl_as2prolog(description(Descr,_),D,head),
	R =  :-(D,property(PropertyID,ID,x)).


%
% Existential property description. See table above
%

owl_as2prolog(description(restriction(_,someValuesFrom(_)),_),false,head) :-  !.

owl_as2prolog(description(restriction(PropertyID,someValuesFrom(Descr)),_),R,body) :-  !,
	owl_as2prolog(description(Descr,y),D,body),
	R =  (D,property(PropertyID,x,y)).

%
% Cardinalities are not handled in this conversion
%

owl_as2prolog(description(restriction(_,maxCardinality(_)),_),false,_) :-  !.
owl_as2prolog(description(restriction(_,minCardinality(_)),_),false,_) :-  !.
owl_as2prolog(description(restriction(_,cardinality(_)),_),false,_) :-  !.


%
% Any other description is taken to be a named class
%

owl_as2prolog(description(Any,X),class(Any,X),_).


%
% Handling of description lists in head and bodies of rules
%

owl_as2prolog(description_list([],_,_),[],_) :-!.

owl_as2prolog(description_list([Descr],X,_),R,body) :- !,
	owl_as2prolog(description(Descr,X),R,body).

owl_as2prolog(description_list([Descr|Rest],X,Separator),T,Param) :-
	owl_as2prolog(description(Descr,X),H,Param),!,	
	owl_as2prolog(description_list(Rest,X,Separator),Tail,Param),
	(   Param = body , ! ,  
	    (H = false, !, T = [false] ; Tail = false, !, T = false
	    ; 
	    T =.. [Separator,H,Tail]
	    ) ; 	    
	T = [H|Tail]
	).

% 
%  Mapping properties. 
%  a. Generate a s(X,Y) :- p(X,Y). for each super property p
%  b. Generate a C(X) :- P(X,Y) for each C in the property domain
%  c. Generate a c(Y) :- p(X,Y) for each range C
%  d. Handle property attributes in process_pt_list predicate
%

owl_as2prolog(property(PID,_Deprecated,_AnnotationList,PID_SuperList,PTList,PID_DomainList,PID_RangeList),L,_) :-
	maplist(map_subproperty,PID_SuperList,L1),
	maplist(map_description(head,_),PID_DomainList,L2),
	maplist(map_description(head,_),PID_RangeList,L3),
	process_pt_list(PID,PTList,L4),
	L = [:-(L1,property(PID,x,y)), :-(L2,property(PID,x,var)), :-(L3,property(PID,var,x))|L4].


% 
%  Mapping individuals
%  a. Generate a C(ID) for each desccription C in the Types list
%  b. Generate a p(ID,Value) for each value declaration in the Property
%  list. 
%

owl_as2prolog(individual(IID,_,TypeList,PropertyList),L,_) :-
       maplist(map_description(fact,IID),TypeList,L1),
       maplist(map_property(IID), PropertyList,L2),
       merge(L1,L2, L).


% 
% Mapping functions (Perform convert operations on each element in a
% list).
% 

map_subclass_prolog(C,D,R) :- 
	owl_as2prolog(subclassOf(C,D),R,_).


map_subproperty(SuperP,property(SuperP,x,y)).

map_description(fact,X,Description,:-(DMap,none)) :- !,
	owl_as2prolog(description(Description,X),DMap,fact).

map_description(Type,X,Description,DMap) :-
	owl_as2prolog(description(Description,X),DMap,Type).

map_property(IID, value(P,V), :-(property(P,IID, V),none)).

	
% 
%  Mappings generated from the attributes of a property.
%  a. Functional and inverse functionals generate a 
%       sameIndividuals(X,Y) :- p(Z,X), P(Z,Y)
%  Transitive: p(X,Z) :- p(X,Y), p(Y,Z). 
%  Symmetric: p(X,Y) :- p(Y,X).
%  Inverse  : p(X,Y) :- inv(Y,X) and inv(X,Y) :- p(Y,X).
%

process_pt_list(PID, [_,F,IF,T,S,iof(Inv)],[Ft,IFt,Tt,St,INVt]) :-
	(   nonvar(F), !, Ft = :-(property(sameIndividuals,x,y), (property(PID,z,x),property(PID,z,y)))
	; Ft = none
	),
	(   nonvar(IF), !, IFt = :-(property(sameIndividuals,x,y), (property(PID,x,z),property(PID,y,z)))
	; IFt = none 
	),
	(   nonvar(T), !, Tt = :-(property(PID,x,z), (property(PID,x,y),property(PID,y,z)))
	; Tt = none
	),
	(   nonvar(S), !, St = :-(property(PID,x,y), property(PID,y,x))
	; St = none
	),
	(   nonvar(Inv), !, INVt = [:-(property(PID,x,y),property(Inv,y,x)),:-(property(Inv,x,y),property(PID,y,x))] 
	; INVt = none
	).



