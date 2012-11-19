% -------------------------------------------------------------------
	
           /************************************************
           *   Predicates to pretty print OWL constructs  *
           ************************************************/

go(StreamName) :-
	open(StreamName,write,Stream),	
%	pp_owl_description(Stream,intersectionOf([a,unionOf([b1,b2]),
%           restriction(pID,allValuesFrom(c)),
%	   restriction(pID2,value('young')),d]),[]),
	owl_pp_class(Stream),
	close(Stream).


owl_pp_class(S) :-
% C = 'http://www.w3.org/2002/03owlt/miscellaneous/consistent001#WhiteWine',
	class(C,Deprecated,CP,AL,DL),
	print(S,'ClassID \t'),print(S,C), nl(S),
        print(S,'Deprecated \t'),print(S,Deprecated),nl(S),
	print(S,'Complete \t'),print(S,CP),nl(S),       
% TODO: Handle empty annotationList
	print(S,'Annotations'),pp_description_list(S,AL,['\t']),
	print(S,'Descriptions'),pp_description_list(S,DL,['\t']),
	fail.

owl_pp_class(_).



% ------------------------------------------------------

pp_owl_description(S,intersectionOf(DL),Tabs) :-
  pp_tabs(S,Tabs),print(S,'intersectionOf'),nl(S),
  pp_description_list(S,DL,['\t'|Tabs]).

pp_owl_description(S,unionOf(DL),Tabs) :-
  pp_tabs(S,Tabs),print(S,'unionOf'),nl(S),
  pp_description_list(S,DL,['\t'|Tabs]).

pp_owl_description(S,complementOf(DL),Tabs) :-
  pp_tabs(S,Tabs),print(S,'complementOf'),nl(S),
  pp_description_list(S,DL,['\t'|Tabs]).

pp_owl_description(S,oneOf(DL),Tabs) :-
  pp_tabs(S,Tabs),print(S,'oneOf'),nl(S),
  pp_description_list(S,DL,['\t'|Tabs]).

pp_owl_description(S,restriction(PropertyID,allValuesFrom(Descr)),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,'Restriction \t'), print(S,PropertyID),print(S,'\t allValuesFrom \t'),nl(S),
  pp_owl_description(S,Descr,['\t','\t','\t'|Tabs]).

pp_owl_description(S,restriction(PropertyID,someValuesFrom(Descr)),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,'Restriction \t'), print(S,PropertyID),print(S,'\t someValuesFrom \t'),nl(S),
  pp_owl_description(S,Descr,['\t','\t','\t'|Tabs]).


pp_owl_description(S,restriction(PropertyID,maxCardinality(Max)),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,'Restriction \t'), 
  print(S,PropertyID),print(S,'\t maxCardinality \t'),print(S,Max),nl(S).

pp_owl_description(S,restriction(PropertyID,minCardinality(Min)),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,'Restriction \t'), 
  print(S,PropertyID),print(S,'\t minCardinality \t'),print(S,Min),nl(S).

pp_owl_description(S,restriction(PropertyID,cardinality(C)),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,'Restriction \t'), 
  print(S,PropertyID),print(S,'\t cardinality \t'),print(S,C),nl(S).

pp_owl_description(S,restriction(PropertyID,value(Value)),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,'Restriction \t'), 
  print(S,PropertyID),print(S,'\t value \t'),print(S,Value),nl(S).

pp_owl_description(S,annotation(APID,Value),Tabs) :-
  pp_tabs(S,Tabs),
  print(S,APID),print(S,'\t'),print(S,Value),nl(S).

pp_owl_description(S,X,Tabs) :-
  pp_tabs(S,Tabs),print(S,X),nl(S).

% ------------------------------------------------------

pp_description_list(_,[],_).

pp_description_list(S,[H|T],Tabs) :-
	pp_owl_description(S,H,Tabs),!,
	pp_description_list(S,T,Tabs).


pp_tabs(_,[]).
pp_tabs(S,[H|T]) :-
  print(S,H),pp_tabs(S,T).

% -----------------------------------------------

find_term(T,X,Y,Z,U) :- 
	owl(X,Y,Z,U),
	(   sub_string(X,_,_,_,T) ; 
	    sub_string(Y,_,_,_,T);
	    Z \= literal(_),sub_string(Z,_,_,_,T) 
	).



