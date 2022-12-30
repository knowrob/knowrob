:- module(logging_messages,
    [ log_error(t),
      log_warning(t),
      log_info(t),
      log_debug(t),
      log_error(t,+),
      log_warning(t,+),
      log_info(t,+),
      log_debug(t,+),
      log_error_and_fail(t),
      log_message(+,t),
      log_message(+,t,+,+)
    ]).
/** <module> Logging messages.

@author Daniel Beßler
@license BSD
*/

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

%% log_error(+Term) is det.
%% log_warning(+Term) is det.
%% log_info(+Term) is det.
%% log_debug(+Term) is det.
%
% Append given term to the logging output.
%
log_error(Term)   :- log_message0(error, Term).
log_warning(Term) :- log_message0(warning, Term).
log_info(Term)    :- log_message0(informational, Term).
log_debug(Term)   :- log_message0(debug, Term).

%% log_error(+Term, +FileIndicator) is det.
%% log_warning(+Term, +FileIndicator) is det.
%% log_info(+Term, +FileIndicator) is det.
%% log_debug(+Term, +FileIndicator) is det.
%
log_error(Term, FileIndicator)   :- log_message1(error,Term,FileIndicator).
log_warning(Term, FileIndicator) :- log_message1(warning,Term,FileIndicator).
log_info(Term, FileIndicator)    :- log_message1(informational,Term,FileIndicator).
log_debug(Term, FileIndicator)   :- log_message1(debug,Term,FileIndicator).

log_message0(Level, Term) :-
    message_to_string(Term, Msg),
    (  source_location(File, Line)
    -> log_message(Level, Msg, File, Line)
    ;  log_message(Level, Msg)
    ).

log_message1(Level, Term, File:Line) :-
    message_to_string(Term, Msg),
    log_message(Level, Msg, File, Line).

%% log_error_and_fail(+Term)
%
% Call log_message/2 and fail afterwards.
%
log_error_and_fail(Term) :- log_message1(error,Term), fail.
