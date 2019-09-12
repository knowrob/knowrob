
:- module(knowrob_entity_queries,
    [
      object_query/4,
      object_queries/2
    ]).

:-  rdf_meta
    object_query(r,?,?,?),
    object_queries(r,?).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Asking queries about objects.

%% object_queries(+Obj:iri, -Queries:list) is det
%
% Gather facts about queries that can be asked about an object.
% Queries are represented as [atom category, atom title, atom query].
% Category and title are primary used for displaying possible queries
% to the user.
%
% @param Obj the object name
% @param Queries list of queries that can be asked about Obj
%
object_queries(Obj, Queries) :-
  findall([Category,Title,Query],
          object_query(Obj,Category,Title,Query),
          QueriesUnsorted),
  sort(QueriesUnsorted, Queries).

%% object_query(+Obj:iri, ?QueryGroup:atom, ?QueryTitle:atom, ?Query:atom) is det
%
% True for objects Obj for which a query exists belonging to the group QueryGroup
% and labeled with QueryTitle.
%
% @param Obj the object name
% @param QueryGroup category of query
% @param QueryTitle name of the query
% @param Query the Prolog-encoded query string
%
object_query(Obj, QueryGroup, QueryTitle, Query) :-
  atom(Obj),
  % queries about specific individuals
  rdf_has(QueryIndividual, knowrob:'queryAbout', Obj),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Obj, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).

object_query(Obj, QueryGroup, QueryTitle, Query) :-
  atom(Obj),
  % queries about specific types
  rdfs_individual_of(Obj, IndividualClass),
  % FIXME: queryAbout some Class is non OWL! use restrictions instead!
  rdf_has(QueryIndividual, knowrob:'queryAbout', IndividualClass),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Obj, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).
