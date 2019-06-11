:- begin_tests(event_graph).

:- use_module(library('knowrob/event_graph')).

test('merge_sequences(equal0)') :-
  event_graph:merge_sequences(
    [a,b],
    [a,b],
    [a,b]).
test('merge_sequences(equal1)') :-
  event_graph:merge_sequences(
    [[a],[b]],
    [[a],[b]],
    [[a],[b]]).
test('merge_sequences(prefix)') :-
  event_graph:merge_sequences(
    [[a]],
    [[a],[b]],
    [[a],[b]]).
test('merge_sequences(suffix)') :-
  event_graph:merge_sequences(
    [[x],[[[a],[b]],[c]]],
    [[y],[[[a],[b]],[c]]],
    [[[[x]],[[y]]],[[[a],[b]],[c]]]).
test('merge_sequences(prefix+suffix_1)') :-
  event_graph:merge_sequences(
    [[a],[x],[[[a],[b]],[c]]],
    [[a],[y],[[[a],[b]],[c]]],
    [[a],[[[x]],[[y]]],[[[a],[b]],[c]]]).
test('merge_sequences(prefix+suffix_2)') :-
  event_graph:merge_sequences2([
    [a,     b,c,   z],
    [a,      d,    z]],
    [a,[[b,c],[d]],z]).
test('merge_sequences(prefix+suffix_3)') :-
  event_graph:merge_sequences(
    [a,b,c],
    [a,  c],
    [a,b,c]).

% here 'z' is always the action that is axiomatized by the ESG
test('esg(d(a,z)') :-
  esg(z,[a],[d(a,z)], [[-z],[-a],[+a],[+z]]).
test('esg(s(a,z))') :-
  esg(z,[a],[s(a,z)], [[-a,-z],[+a],[+z]]).
test('esg(<(a,z),>(a,z))') :-
  catch(esg(z,[a],[a<z,a>z],_),
    model_error(_,<(_,_)),
    true).
test('esg(f(a,z))') :-
  esg(z,[a],[f(a,z)], [[-z],[-a],[+a,+z]]).
test('esg(a<b,s(a,z),f(b,z))') :-
  esg(z,[a,b],[a<b,s(a,z),f(b,z)],
      [[-a,-z],[+a],[-b],[+b,+z]]).
test('esg(m(a,b),mi(b,a),s(a,z),f(b,z))') :-
  esg(z,[a,b],[m(a,b),mi(b,a),s(a,z),f(b,z)],
      [[-a,-z],[+a,-b],[+b,+z]]).
test('esg(m(a,b),si(z,a),fi(z,b))') :-
  esg(z,[a,b],[m(a,b),si(z,a),fi(z,b)],
      [[-a,-z],[+a,-b],[+b,+z]]).
test('esg(m(a,b),fi(z,b),si(z,a))') :-
  esg(z,[a,b],[m(a,b),fi(z,b),si(z,a)],
      [[-a,-z],[+a,-b],[+b,+z]]).
test('esg(a<b,a<c,b<c,s(a,z),f(c,z))') :-
  esg(z,[a,b,c],[a<b,a<c,b<c,s(a,z),f(c,z)],
      [[-a,-z],[+a],[-b],[+b],[-c],[+c,+z]]).
test('esg(o(a,z),o(b,a),o(b,z))') :-
  esg(z,[a,b],[o(a,z),o(b,a),o(b,z)],
      [[-b],[-a],[-z],[+b],[+a],[+z]]).

test('esg_truncated(s(a,z))') :-
  esg_truncated(z,[a],[s(a,z)], [
      [[-z],[-a],[+a],[+z]],[],[]]).
test('esg_truncated(o(z,a))') :-
  esg_truncated(z,[a],[o(z,a)],[
      [[-z],[-a],[+z]],[],[[+a]]]).
test('esg_truncated(o(a,z))') :-
  esg_truncated(z,[a],[o(a,z)],[
      [[-z],[+a],[+z]],[[-a]],[]]).
test('esg_truncated(o(a,z),o(b,a),o(b,z))') :-
  esg_truncated(z,[a,b],[o(a,z),o(b,a),o(b,z)],[
      [[-z],[+b],[+a],[+z]],[[-a],[-b]],[]]).

test('esg(GraspLift)') :-
  esg(gl,[g,l],[<(g,l),>(l,g),si(gl,g),fi(gl,l)],
      [[-g,-gl],[+g],[-l],[+l,+gl]]).
test('esg_truncated(GraspLift)') :-
  esg_truncated(gl,[g,l],[>(l,g),<(g,l),fi(gl,l),si(gl,g)],[
      [[-gl],[-g],[+g],[-l],[+l],[+gl]],[],[]]).

test('esg_pop(any)') :-
  findall([E,G],
    esg_pop([[-a,-z],[+a,-b],[+b,+z]],E,G),
    Es),
  once(member([-a,[[-z],[+a,-b],[+b,+z]]],Es)),
  once(member([-z,[[-a],[+a,-b],[+b,+z]]],Es)).
test('esg_pop(-a)') :-
  esg_pop(
    [[-a,-z],[+a,-b],[+b,+z]],
    -a,
    [[-z],[+a,-b],[+b,+z]]).
test('esg_pop(+a,fail)', fail) :-
  esg_pop(
    [[-a,-z],[+a,-b],[+b,+z]],
    +a,
    [[-z],[+a,-b],[+b,+z]]).
test('esg_pop(nested1)') :-
  findall([E,G],
    esg_pop([
      [[[-a,-r],[+a,+r]],
       [[-t],[+t]]],[-b],[+b,+z]],E,G),
    Es),
  % three possible next endpoints -a,-r,-t
  once(member([-a,[[[[-r],[+a,+r]],
       [[-t],[+t]]],[-b],[+b,+z]]],Es)),
  once(member([-r,[[[[-a],[+a,+r]],
       [[-t],[+t]]],[-b],[+b,+z]]],Es)),
  once(member([-t,[[[[+t]],
       [[-a,-r],[+a,+r]]],[-b],[+b,+z]]],Es)).
test('esg_pop(nested2)') :-
  findall([E,G],
    esg_pop([
      [[[-a,-r],[+a,+r]],
       [[-t]]],[-b],[+b,+z]],E,G),
    Es),
  % three possible next endpoints -a,-r,-t
  once(member([-a,[[[[-r],[+a,+r]],
       [[-t]]],[-b],[+b,+z]]],Es)),
  once(member([-r,[[[[-a],[+a,+r]],
       [[-t]]],[-b],[+b,+z]]],Es)),
  once(member([-t,[[
       [[-a,-r],[+a,+r]]],[-b],[+b,+z]]],Es)).

test('path_to_endpoint(-b)') :-
  event_graph:path_to_endpoint(
    [[-a,-z],[+a],[-b],[+b,+z]],
    -b,
    [[-a,-z],[+a]],
    [[-b],[+b,+z]]).
test('path_to_endpoint(+b)') :-
  event_graph:path_to_endpoint(
    [[-a,-z],[+a],[-b],[+b,+z]],
    +b,
    [[-a,-z],[+a],[-b]],
    [[+b,+z]]).
test('path_to_endpoint(+k,fail)',fail) :-
  event_graph:path_to_endpoint(
    [[-a,-z],[+a],[-b],[+b,+z]],
    +k,_,_).

test('esg_join(insert1)') :-
  esg_join(
    [[-z],[+z]],
    [z,[[-z],[-b],[+z]]],
    [[-z],[-b],[+z]]).
test('esg_join(insert2)') :-
  esg_join(
    [[-z],[+z],[-k]],
    [z,[[-z],[-b],[+z]]],
    [[-z],[-b],[+z],[-k]]).
test('esg_join(parallel path)') :-
  esg_join(
    [[-z],[-a],[+a],[+z]],
    [z,[[-z],[-b],[+b],[+z]]],
    [[-z],[[[-b],[+b]],[[-a],[+a]]],[+z]]).

:- end_tests(event_graph).
