/*  tokenizer.pl

    Author:        Daniel Beßler
    E-mail:        danielb@informatik.uni-bremen.de
    WWW:           http://www.ease.org
    Copyright (C): 2018, University of Bremen

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    As a special exception, if you link this library with other files,
    compiled with a Free Software compiler, to produce an executable, this
    library does not by itself cause the resulting executable to be covered
    by the GNU General Public License. This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

:- module(tokenizer, [
     tokenize/2
]).
/** <module> Tokenizing events for activity parsing.

Tokens are 4-ary terms of the form:
  tok(Instant,Event,{+,-}(Type),[Participants])
Tokenization of events is pretty simple. Each event generates
two tokens: one for its start (+) and one for its end (-).
Addtionally, participants and regions associated to the event
are represented in the token.

@author Daniel Beßler
@license GPL
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('knowrob/temporal')). % `interval/2`

:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

:- rdf_meta tokenize(r,t).

interval_(Event, [Begin,End]) :- interval(Event,[Begin,End]),!.
interval_(Event, [Begin,End]) :-
  % KnowRob's `interval/2` does not work with dul:'TimeInterval' dul yet
  rdf_has(Event,dul:hasTimeInterval,Interval),
  rdf_has_prolog(Interval, allen:hasIntervalBegin, Begin),
  rdf_has_prolog(Interval, allen:hasIntervalEnd, End).

%% tokenize(+Episode,?Tokens).
%
% Tokenize given episode.
% The structure of the episode symbol is that it links
% all events via knowrob:subAction.
% These are collected and mapped to a token representation.
%
% @param Episode VR episode symbol.
% @param Tokens Tokenized episode.
%
tokenize(Episode,Tokens) :-
  findall(Event, rdf_has(Episode,dul:hasConstituent,Event), Events),
  tokenize_(Events,Unsorted),
  filter_tokens(Unsorted,Filtered),
  % first argument of each token is time such that we can use Prolog builtin `sort`
  sort(Filtered,Tokens).

filter_tokens([],[]) :- !.
filter_tokens([Tok|Rest],[Tok|RestFiltered]) :-
  filter_tokens(Rest,RestFiltered).

tokenize_([], []).
tokenize_([Evt|Rest],[Tok1,Tok2|RestTokens]) :-
  tokenize_event(Evt,Tok1,Tok2),
  tokenize_(Rest,RestTokens).

tokenize_event(Event,
    tok(Begin,Event,-(EvtType),Participants),
    tok(End,  Event,+(EvtType),Participants)) :-
  rdfs_individual_of(Event,dul:'Event'), !,
  % Event types are disjoint
  % TODO: is this really safe to assume? e.g., artifact contact and effector contact.
  %       this needs special handling because different constituents may refer to
  %       different event types.
  %       Maybe would be better to stick to event individual in endpoints?
  once((
    rdf(Event,rdf:type,EvtType),
    rdfs_subclass_of(EvtType,dul:'Event')
  )),
  findall(P, rdf_has(Event, dul:hasParticipant, P), Participants),
  interval_(Event, [Begin,End]).

tokenize_event(Event,_,_) :-
  % should not happen, everything should be of type 'Event'.
  % If not this would indicate some unknown event type in the log which is
  % not mapped to flanagan ontology.
  throw(error(type_error(dul:'Event',Event), _)).
