:- module(logging_messages,
    [ log_message(+,t),
      log_error(t),
      log_warning(t),
      log_info(t),
      log_debug(t)
    ]).

:- multifile prolog:message//1.

%% log messages
prolog:message(log(Term)) -->
	prolog:message(Term).
prolog:message(log(Term)) -->
	{ term_to_atom(Term, Atom) },
	[ Atom ].

%% common error messages
prolog:message(exception(Term)) -->
	[ 'An exception has been thrown: ' ],
	prolog:message(log(Term)).
prolog:message(type_error(ValidType,Culprit)) -->
	[ 'An argument (~w) does not meet the required type (~w)'-[Culprit,ValidType] ].

%% kb/1 messages
prolog:message(kb(initialization(started))) -->
	[ 'The knowledge base is starting up, that may take a few moments!' ].
prolog:message(kb(initialization(finished))) -->
	[ 'The knowledge base has finished starting up!' ].

%%
%
log_message(Level,Term) :-
	print_message(Level,log(Term)).

%%
%
log_error(Term)   :- log_message(error,Term).
log_warning(Term) :- log_message(warning,Term).
log_info(Term)    :- log_message(informational,Term).
log_debug(Term)   :- log_message(debug(kb),Term).
