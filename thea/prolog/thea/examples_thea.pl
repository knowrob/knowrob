% :- use_module('../owl_parser.pl').
% :- use_module('../owl_as2rdf.pl').
% :- use_module('../owl_reasoner.pl').
% 


go_thea(InFile,OutFile,thea_as) :-
	owl_parse(InFile,complete,complete,false),
	current_output(CO),
	open(OutFile,write,S), set_output(S),
	listing(class/5),
	listing(subclassOf/2),
	listing(property/7),
	listing(individual/4),
	listing(equivalentSet/1),
	listing(sameIndividuals/1),

	close(S),
	set_output(CO).

go_thea(InFile,OutFile,dlp) :-
	owl_parse(InFile,complete,complete,false),
	current_output(CO), 
	open(OutFile,write,S), set_output(S), 

	print('% ---------------- Individuals ---------'),nl, 
	findall(_,(individual(IID,_,TypeList,PropertyList),
		   owl_as2prolog(individual(IID,_,TypeList,PropertyList),Options)),_),
 
	print('% ---------------- Classes ---------'),nl, 
	findall(_,(class(C,_,CP,_,DL),owl_as2prolog(class(C,_,CP,_,DL),Options)),_),


	print('% ---------------- Subclasses ---------'),nl, 
	findall(_,(subclassOf(C,D), owl_as2prolog(subclassOf(C,D),Options)),_), 

	print('% ---------------- Properties ---------'),nl, 
	findall(_,(property(PID,Deprecated,AnnotationList,PID_SuperList,PTList,PID_DomainList,PID_RangeList),
		   owl_as2prolog(property(PID,Deprecated,AnnotationList,PID_SuperList,PTList,PID_DomainList,PID_RangeList),Options)),
		_),
    
	close(S),
	set_output(CO).


go_thea(InFile,_OutFile,dig_tell) :-
	owl_parse(InFile,complete,complete,false),!,
	assert(owl_reasoner:dig_kb(reasoner_kb,1)), % dummy entry to bypass reasoner 
	dig_tell_all('http://localhost:8088',reasoner_kb,Response),print(Response),nl.
	% rename_file('_thea_dir_request.xml',OutFile).


go_dig(KBName,Response) :-
	working_directory(_,'c:/documents and settings/vangelis vassiliadis/my documents/'),
	owl_parse('c:/Program Files/Protege_3.2/dig-test.owl',complete,complete,false),
	dig_new_kb('http://localhost:8088',KBName,_Result),
	dig_tell_all('http://localhost:8088',KBName,Response),print(Response),nl.
	


dig_tell_all(ReasonerURL,KBName,X) :- 
	findall(Rc,(class(C1,C2,C3,C4,C5), owl_as2dig(class(C1,C2,C3,C4,C5),Rc)),LRC),
	findall(Rsc,(subclassOf(SC1,SC2), owl_as2dig(subclassOf(SC1,SC2),Rsc)),LRSC),
	findall(Ri,(individual(A,B,C,D),owl_as2dig(individual(A,B,C,D),Ri)),LRI),
	findall(Rp,(property(A1,A2,A3,A4,A5,A6,A7),owl_as2dig(property(A1,A2,A3,A4,A5,A6,A7),Rp)),LRP),
	flatten([LRC,LRSC,LRI,LRP],RF8), 

/*      	
	individual(A,B,C,D),owl_as2dig(individual(A,B,C,D),R,_),
	property(A1,A2,A3,A4,A5,A6,A7),
	owl_as2dig(property(A1,A2,A3,A4,A5,A6,A7),Ra,_),print(Ra),nl,nl,nl,
*/
	dig_tell(ReasonerURL,KBName,RF8,X).



reason_test :-
/*
	owl_as2prolog(class(a,_,complete,_,[intersectionOf([b,unionOf([c1,c2,c3]),d])]),R1,_), owl_as2prolog(R1), read(_),
	owl_as2prolog(class(a,_,complete,_,[b]),R2,_),owl_as2prolog(R2),read(_),
	owl_as2prolog(class(a,_,partial,_,[restriction(p,allValuesFrom(b))]),R3,_),owl_as2prolog(R3),read(_),
	owl_as2prolog(class(a,_,partial,_,[b]),R4,_),owl_as2prolog(R4),read(_),

	owl_as2prolog(class(a,_,complete,_,[intersectionOf([b,restriction(p,value(v))])]),R5,_),owl_as2prolog(R5),read(_),
*/
	owl_as2prolog(class(myclass,_,complete,_,[intersectionOf([a,b,restriction(pid,allValuesFrom(z))])]),[]),

	owl_as2prolog(class(myclass,_,complete,_,[intersectionOf([a,b,restriction(pid,someValuesFrom(z))])]),[]),

/*
	owl_as2prolog(property(myproperty,_,_,[s1,s2,s3],[_,functional,inversefunctional,transitive,symmetric,iof(inv)],[d1,d2,d3], [r1,r2,r3]),R8,_),
	owl_as2prolog(R8), nl, read(_),

	owl_as2prolog(individual(peter, _, [unionOf([a,b,restriction(p,value(v))])], [value(p1,v1),value(p2,v2)]),R9,_),
	owl_as2prolog(R9), nl, read(_),
*/
	owl_as2prolog(individual(peter, _, [intersectionOf([a,b,c]),restriction(pr, allValuesFrom(d))], [value(p1,v1),value(p2,v2)]),[]).

relates(I1,P,I2) :-
	individual(I1,_,_,PList),
	member(value(P,I2),PList).

