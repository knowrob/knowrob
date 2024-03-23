:- module(unittest,
    [ test_and_report/2,
      assert_true/1,
      assert_false/1,
      assert_equals/2,
      assert_unifies/2
    ]).
/** <module> Run plunit tests and report results.
This needs to be done by intercepting the messages sent by plunit, and converting
them into facts. This is necessary to be able to generate reports in different
formats, and to be able to run tests in parallel.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'), [ rdf_meta/1 ]).
:- use_module(library(plunit)).
:- use_module('messages').
:- expects_dialect(sicstus). % for use_module/3

:- dynamic test_suite_begin/2,
           test_suite_end/2,
           test_case_begin/5,
           test_case_end/3,
           test_case_failure/3,
           out_stream_/1.

:- rdf_meta((
		assert_true(t),
		assert_false(t),
		assert_equals(t,t),
		assert_unifies(t,t))).

:- module_transparent
        assert_true/1,
        assert_false/1,
        assert_equals/2,
        assert_unifies/2.

%% assert_true(+Goal) is det.
%
% Assert that Goal holds.
% Supposed to be used withing unit tests.
%
% @param Goal the goal to be tested.
%
assert_true(Goal) :- assertion(Goal).

%% assert_false(+Goal) is det.
%
% Assert that Goal does not hold.
% Supposed to be used withing unit tests.
%
% @param Goal the goal to be tested.
%
assert_false(Goal) :- assertion(\+ Goal).

%% assert_equals(+A,+B) is det.
%
% Assert that A and B are instantited to the same value.
% Supposed to be used withing unit tests.
%
% @param Goal the goal to be tested.
%
assert_equals(A,B) :- assertion(A == B).

%% assert_unifies(+A,+B) is det.
%
% Assert that A and B can be unified.
% Supposed to be used withing unit tests.
%
% @param Goal the goal to be tested.
%
assert_unifies(A,B) :- assertion(A = B).

%% pretty print some messages
prolog:message(test_failed(Unit, Name, Error)) -->
	[ '[plunit] ~p:~p failed. msg:"~p"'-[Unit,Name,Error] ].
prolog:message(test_nondet(Name)) -->
	[ '[plunit] ~p succeeded with choicepoint'-[Name] ].
prolog:message(test_report(name=N,tests=C0,failures=C1,errors=C2,time=Time)) -->
	[ '[plunit] ~p pass:~p/~p time:~p'-[N,Count,C0,Time] ],
	{ Count is C0 - (C1 + C2) }.
prolog:message(test_blocked(Name,Msg)) -->
    [ '[plunit] test "~p" is blocked: ~p'-[Name,Msg] ].
prolog:message(test_fixme(num_tests(1)))   --> [ '[plunit] 1 test is labeled as `fixme`' ].
prolog:message(test_fixme(num_tests(Num))) --> [ '[plunit] ~p tests are labeled as `fixme`'-[Num] ].
prolog:message(run_tests(Module)) -->
    [ '[plunit] Testing module "~p".'-[Module] ].

%% Intercept plunit messages, and
%% create facts using the dynamic "test_*" predicates.
%user:message_hook(MsgTerm, _Level, _Lines) :-
%    writeln(message_hook(MsgTerm)), fail.
user:message_hook(plunit(Args), _Level, _Lines) :-
	once( plunit_message_hook(Args) ).

% suppress some messages
plunit_message_hook(test_files(_,_)) :- !.
plunit_message_hook(all_passed(_)) :- !.
plunit_message_hook(all_passed(_,_,_)) :- !.
plunit_message_hook(progress(_,_,_)) :- !.
plunit_message_hook(progress(_,_,_,_)) :- !.
plunit_message_hook(blocked(_)) :- !.

plunit_message_hook(failed_assertions(_)) :- !.
plunit_message_hook(failed(_)) :- !.
plunit_message_hook(passed(_)) :- !.
plunit_message_hook(total_time(_)) :- !.

% uncomment this if something breaks to see what messages are being sent
% by plunit. Every now and then the format of the messages changes.
%plunit_message_hook(MsgTerm) :-
%    writeln(MsgTerm), fail.

plunit_message_hook(blocked(FileIndicator, Name, Msg)) :-
    log_warning(test_blocked(Name, Msg), FileIndicator).
plunit_message_hook(fixme(0, 0, 0)) :- !.
plunit_message_hook(fixme(Failed, Passed, Nondet)) :-
    Total is Failed + Passed + Nondet,
    % TODO: add file indicator to log. Unfortunately the msg does not carry context. Same below.
    log_warning(test_fixme(num_tests(Total))).
plunit_message_hook(no_tests) :-
    log_warning(no_tests).

plunit_message_hook(begin(Unit)) :-
	get_time(Time), assertz(test_suite_begin(Unit,Time)).
plunit_message_hook(begin(Unit:Test, File:Line, _STO)) :-
	unpack_test_(Test,TestA),
	get_time(Time), assertz(test_case_begin(Unit,TestA,Time,File,Line)).

plunit_message_hook(end(Unit)) :-
	get_time(Time), assertz(test_suite_end(Unit,Time)).
plunit_message_hook(end(Unit, _Report)) :-
	get_time(Time), assertz(test_suite_end(Unit,Time)).
plunit_message_hook(end(Unit:Test, _File:_Line, _STO)) :-
	unpack_test_(Test,TestA),
	get_time(Time), assertz(test_case_end(Unit,TestA,Time)).

plunit_message_hook(failed(Unit, Name, Line, _ErrorTerm, _Time, FailedAssert)) :-
	plunit_message_hook(failed(Unit, Name, Line, FailedAssert)).
plunit_message_hook(failed(Unit, Name, Line, Error, _Time)) :-
	plunit_message_hook(failed(Unit, Name, Line, Error)).
plunit_message_hook(failed(Unit:Name, _Counter, Line, Error)) :-
	% seems format has changed in newer versions of plunit, it starts with "Unit:Name" which
	% was not the case before afaik.
	plunit_message_hook(failed(Unit, Name, Line, Error)).
plunit_message_hook(failed(Unit, Name, _Line, Error)) :-
	% need to select the output stream for print_message explicitely in the
	% the scope of *run_tests*.
	%out_stream_(OS),
	open_null_stream(OS),
	with_error_to_(OS,
		print_message(warning,test_failed(Unit, Name, Error))),
	close(OS),
	assertz(test_case_failure(Unit,Name,Error)).
plunit_message_hook(failed_assertion(Unit, Name, Line, _Error, _STO, Reason, Goal)) :-
	% get tests options
	plunit:current_unit(Unit, Module, _Supers, _UnitOptions),
	Module:'unit test'(Name, Line, Options, _Body),
	% ignore marked tests
	(	option(fixme(_), Options)
	;	(
		% need to select the output stream for print_message explicitely in the
		% the scope of *run_tests*.
		%out_stream_(OS),
		open_null_stream(OS),
		Error=failed_assertion(Reason,Goal),
		with_error_to_(OS,
			print_message(warning,test_failed(Unit, Name, Error))),
		close(OS),
		assertz(test_case_failure(Unit,Name,Error))
	)).
plunit_message_hook(nondet(_,_,Name)) :-
	print_message(warning,test_nondet(Name)).

%% NOTE: @(Test,Args) is used when *forall* is used in tests options.
unpack_test_(@(Test,_),Test) :- !.
unpack_test_(Test,Test) :- !.

%%
is_plt_file(File) :-
	file_name_extension(_, plt, File).

%%
has_test_file(File) :-
	file_name_extension(X, pl, File),
	file_name_extension(X, plt, File0),
	exists_file(File0).

%%
is_pl_test_file(File) :-
	file_name_extension(_, pl, File),
	\+ has_test_file(File),
	\+ file_base_name(File, '__init__.pl').

%% test_and_report(+Target, +Opts) is det.
%
% Runs tests corresponding to Target.
% Opts is a list of options that may hold the following
% keys:
%
% - xunit(File): write XUnit output to given file.
%
% - report: write a report once all tests finished.
%
test_and_report(Target, _Opts) :-
	\+ atom(Target), !,
	throw(invalid_argument(test_and_report,Target)).

test_and_report(Target, Opts) :-
	%% run tests and report
	setup_call_cleanup(
		%% setup
		true,
		%% call
		( test_and_report1(Target,Opts)
		, ( memberchk(xunit(File),Opts)      -> test_report_xunit_(File) ; true )
		, ( memberchk(xunit_term(Term),Opts) -> test_report_xunit_term_(Term) ; true )
		),
		%% cleanup
		( test_suite_retract_ )
	).

test_and_report1(Target, Opts) :-
	exists_file(Target),!,
	run_tests_(Target,Opts).

test_and_report1(Target,Opts) :-
	exists_directory(Target),!,
	run_tests_(Target,Opts).

%%
run_tests_(Directory,Opts) :-
	exists_directory(Directory),!,
	directory_files(Directory,Entries),
	forall(
		( member(Entry,Entries), \+ atom_prefix(Entry,'.') ),
		( atomic_list_concat([Directory,Entry],'/',Child),
			( exists_directory(Child) -> run_tests_(Child,Opts)
			; is_plt_file(Child)      -> run_test_(Child,Opts)
			; is_pl_test_file(Child)  -> run_test_(Child,Opts)
			; true )
		)
	).

run_tests_(Target,Opts) :-
	run_test_(Target,Opts)
	-> true
	;  throw(invalid_argument(run_tests_,Target)).

%%
run_test_(TestFile, Opts) :-
	file_name_extension(X, plt, TestFile),!,
	file_name_extension(X, pl,  ModuleFile),
	run_test_(ModuleFile, Opts).

run_test_(ModuleFile, Opts) :-
	file_name_extension(_, pl,  ModuleFile),
	%%
	%directory_file_path(FileDir, _, ModuleFile),
	%( get_package_path_(FileDir,PackagePath)
	%-> init_ros_package(PackagePath)
	%;  true
	%),
	%%
	catch(
		run_test__(ModuleFile, Opts),
		Error,
		once(
			( Error=error(existence_error(unit_test,_),_)
			; Error=error(permission_error(load,source,_),_)
			; print_message(error,test_failed(ModuleFile, '*', Error))
			)
		)
	).

run_test__(ModuleFile, Opts) :-
	% call use_module in case the file was not loaded before.
	% this is important for modules that are not auto-loaded in
	% the __init__.pl of the package.
	( source_file(ModuleFile)
	-> true
	;  use_module(ModuleFile)
	),
	% get the module name
	use_module(Module,ModuleFile,[]),
	log_info(run_tests(Module)),
	% load plt file if any
	load_test_files(_),
	% remember old user output
	stream_property(OldOut, alias(user_output)),
	retractall(out_stream_(_)),
	assertz(out_stream_(OldOut)),
	catch(
		ignore(run_tests([Module])),
		Error,
		assert_run_tests_error(Module, ModuleFile, Error)),
	ignore((memberchk(silent,Opts) -> true ; test_report_console_(Module))).

%%
assert_run_tests_error(Module, ModuleFile, Error) :-
    plunit_message_hook(begin(Module)),
    plunit_message_hook(begin(Module:'*', ModuleFile:1, _)),
    plunit_message_hook(failed(Module, '*', 1, Error)),
    plunit_message_hook(end(Module:'*', ModuleFile:1, _)),
    plunit_message_hook(end(Module)).

%%
/*
get_package_path_(Directory,PkgPath) :-
	atomic_list_concat(Entries,'/',Directory),
	append(PrefixEntries,[src|Suffix],Entries),
	\+ memberchk(src, Suffix),
	append(PrefixEntries,[src,''],X),
	atomic_list_concat(X,'/',PkgPath),!.
*/

%% retract dynamic facts
test_suite_retract_ :-
	retractall(test_suite_begin(_,_)),
	retractall(test_suite_end(_,_)),
	retractall(test_case_begin(_,_,_,_,_)),
	retractall(test_case_end(_,_,_)),
	retractall(test_case_failure(_,_,_)).

%% make a call but redirect *user_error* to another stream.
with_error_to_(QueryStage,Goal) :-
	stream_property(OldErr, alias(user_error)),
	set_stream(QueryStage, alias(user_error)),
	call(Goal),
	set_stream(OldErr, alias(user_error)).

%%
test_report_(Opts) :-
	member(xunit(File),Opts),
	test_report_xunit_(File).

test_report_console_(Module) :-
	xunit_term_(Module,element(testsuite,Args,_Body)),
	X=..[test_report|Args],
	print_message(informational,X),
	% force printing report to console
	phrase(prolog:message(X), [MsgPattern-MsgArgs]),
	format(atom(MsgAtom),MsgPattern,MsgArgs),
	writeln(MsgAtom).

%%
test_report_xunit_term_(Term) :-
    xunit_term_(_Module,element(testsuite,_Args,Body)),
    member(Term,Body).

%%
test_report_xunit_(File) :-
	findall(Term,
		xunit_term_(_,Term),
		Terms
	),
	test_report_num_tests(Terms,NumTests),
	test_report_num_failures(Terms,NumFailures),
	test_report_time(Terms,TimeTotal),
	open(File,write,QueryStage),
	xml_write(QueryStage,
		element(testsuites,
			[ name='plunit',
			  tests=NumTests,
			  failures=NumFailures,
			  time=TimeTotal
			],
			Terms
		),
		[layout(true)]),
	close(QueryStage).

%%
test_report_num_tests([],0) :- !.
test_report_num_tests([X|Xs],Count) :-
	X=element(testsuite,Args,_),
	member((=(tests,Count0)),Args),
	test_report_num_tests(Xs,Count1),
	Count is Count0 + Count1.

%%
test_report_num_failures([],0) :- !.
test_report_num_failures([X|Xs],Count) :-
	X=element(testsuite,Args,_),
	member((=(failures,Count0)),Args),
	test_report_num_failures(Xs,Count1),
	Count is Count0 + Count1.

%%
test_report_time([],0.0) :- !.
test_report_time([X|Xs],Time) :-
	X=element(testsuite,Args,_),
	member((=(time,Time0)),Args),
	test_report_time(Xs,Time1),
	Time is Time0 + Time1.

% XUnit term generator
xunit_term_(Module, element(testsuite,
		[ name=Module, tests=NumTests,
		  failures=NumFailures, errors=NumErrors,
		  time=TestTime ], TestCaseTerms)) :-
	%%
	once(test_suite_begin(Module,T0)),
	once(test_suite_end(Module,T1)),
	TestTime is T1 - T0,
	%%
	findall(X0, test_case_begin(Module,X0,_,_,_), TestCases),
	length(TestCases,NumTests),
	NumTests > 0,
	%%
	findall(X1, (
		test_case_failure(Module,X1,Failure),
		xunit_is_failure_(Failure)
	), Failures0),
	list_to_set(Failures0,Failures),
	length(Failures,NumFailures),
	%%
	findall(X2, (
		test_case_failure(Module,X2,Err),
		xunit_is_error_(Err)
	), Errors0),
	list_to_set(Errors0,Errors),
	length(Errors,NumErrors),
	%%
	findall(TestTerm, (
		member(X3, TestCases),
		xunit_test_term_(Module,X3,TestTerm)
	), TestCaseTerms).

xunit_test_term_(Module,TestCase,
	element(testcase,
		[ name=TestCase, file=File, line=Line, time=TestTime ],
		FailureTerms)) :-
	%%
	test_case_begin(Module,TestCase,T0,File,Line),
	test_case_end(Module,TestCase,T1),
	TestTime is T1 - T0,
	%%
	findall(FailureTerm, (
		test_case_failure(Module,TestCase,Failure),
		xunit_failure_term_(Failure,FailureTerm)
	), FailureTerms).

%%
xunit_is_failure_(failed).
xunit_is_failure_(succeeded(_)).
xunit_is_error_(X) :- \+ xunit_is_failure_(X).

%%
xunit_failure_term_(failed,
	element(failure, [ type=failed, message=Msg ], [Txt])) :-
	Msg='goal failed',
	Txt=Msg,
	!.

xunit_failure_term_(succeeded(_),
	element(failure, [ type=failed, message=Msg ], [Txt])) :-
	Msg='goal succeeded but should have failed',
	Txt=Msg,
	!.

xunit_failure_term_(Error,
	element(failure, [ type=error, message=Msg ], [Txt])) :-
	( atom(Error)
	-> Msg = Error
	% TODO: not allowed to keep the term in case of XML output? it would be nice for term output
	;  term_to_atom(Error,Msg)
	),
	Txt=Msg.
