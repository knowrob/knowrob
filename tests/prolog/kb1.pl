:- module(kb1,
    [ woman/1,
      loves/2,
      jealous/2
    ]).

woman(mia).
woman(jody).
woman(yolanda).

loves(vincent,mia).
loves(marsellus,mia).
loves(pumpkin,honey_bunny).
loves(honey_bunny,pumpkin).

jealous(X, Y) :-
    loves(X, Z),
    loves(Y, Z),
    X \== Y.

