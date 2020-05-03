
% query language
:- use_module('./query.pl').
:- use_module('./computable.pl').
%:- use_module('./designator.pl').
:- use_module('./export.pl').

% query terms
:- use_module('terms/holds').
:- use_module('terms/is_a').
:- use_module('terms/is_at').
:- use_module('terms/occurs').

% query scoping
:- use_module('scopes/temporal').
%:- use_module('scopes/situational').
