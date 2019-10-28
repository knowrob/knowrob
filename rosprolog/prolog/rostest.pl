/*
  Copyright (C) 2019 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(rostest,
    [
      rospl_run_tests/2
    ]).
/** <module> Run plunit tests in the scope of a ROS workspace.

@author Daniel Beßler
@license BSD
*/

:- use_module(library(plunit)).

:- dynamic test_suite_begin/2,
           test_suite_end/2,
           test_case_begin/3,
           test_case_end/3,
           test_case_failure/3,
           out_stream_/1.

%% pretty print some messages
prolog:message(test_failed(Unit, Name, Error)) -->
    [ '[plunit] ~p failed:~p msg:"~p"'-[Unit,Name,Error] ].
prolog:message(test_report(name=N,tests=C0,failures=C1,errors=C2,time=Time)) -->
    [ '[plunit] ~p pass:~p/~p time:~p'-[N,Count,C0,Time] ],
    { Count is C0 - (C1 + C2) }.

%% Intercept plunit messages, and
%% create facts using the danymic "test_*" predicates.
user:message_hook(plunit(Args), _Level, _Lines) :-
  once( plunit_message_hook(Args) ).
plunit_message_hook(begin(Unit)) :-
  get_time(Time), assertz(test_suite_begin(Unit,Time)).
plunit_message_hook(end(Unit)) :-
  get_time(Time), assertz(test_suite_end(Unit,Time)).
plunit_message_hook(begin(Unit:Test, _File:_Line, _STO)) :-
  get_time(Time), assertz(test_case_begin(Unit,Test,Time)).
plunit_message_hook(end(Unit:Test, _File:_Line, _STO)) :-
  get_time(Time), assertz(test_case_end(Unit,Test,Time)).
plunit_message_hook(failed(Unit, Name, _Line, Error)) :-
  % need to select the output stream for print_message explicitely in the
  % the scope of *run_tests*.
  out_stream_(OS),
  with_error_to_(OS,
    print_message(information,test_failed(Unit, Name, Error))),
  assertz(test_case_failure(Unit,Name,Error)).

%% rospl_run_tests(+Target, +Opts) is det.
%
% Runs tests corresponding to Target.
% Target must be a term "Pkg:Module" where
% Pkg is the name of a ROS package, and Module
% is the path to a Prolog module relative to
% the *prolog* directory of that package.
% Opts is a list of options that may hold the following
% keys:
%
% - xunit(File): write XUnit output to given file.
%
% - report: write a report once all tests finished.
%
rospl_run_tests(Pkg:Module, Opts) :-
  % enforce atom
  ( atom(Module) -> ModAtom = Module ; term_to_atom(Module,ModAtom) ),
  setup_call_cleanup(true,
    ( rospl_run_tests_(Pkg, ModAtom, Opts) ),
    ( test_suite_retract_(ModAtom) )
  ).

rospl_run_tests_(Pkg, Module, Opts) :-
  % load files
  use_package_module_(Pkg,Module),
  load_test_files(_),
  % remember old user output
  stream_property(OldOut, alias(user_output)),
  retractall(out_stream_(_)),
  assertz(out_stream_(OldOut)),
  % silence plunit (avoid that it displays dots)
  open_null_stream(NullStream),
  % finall call *rubn_tests*
  with_error_to_(NullStream, ignore(run_tests([Module]))),
  close(NullStream),
  % make a custom report
  forall(test_report_(Module,Opts),true).

%% retract dynamic facts
  retractall(test_suite_begin(Module,_)),
  retractall(test_suite_end(Module,_)),
  retractall(test_case_begin(Module,_,_)),
  retractall(test_case_end(Module,_,_)),
  retractall(test_case_failure(Module,_,_)).

%% make a call but redirect *user_error* to another stream.
with_error_to_(Stream,Goal) :-
  stream_property(OldErr, alias(user_error)),
  set_stream(Stream, alias(user_error)),
  call(Goal),
  set_stream(OldErr, alias(user_error)).

%%
use_package_module_(Pkg,Module) :-
  register_ros_package(Pkg,_PkgPath),
  use_module(library(Module)).

%%
test_report_(Module,Opts) :-
  member(xunit(File),Opts),
  test_report_xunit_(Module,File).

test_report_(Module,Opts) :-
  member(report,Opts),
  xunit_term_(Module,element(testsuite,Args,_Body)),
  X=..[test_report|Args],
  print_message(information,X).

%%
test_report_xunit_(Module,File) :-
  xunit_term_(Module,Term),
  open(File,write,Stream), 
  xml_write(Stream, [Term], [layout(true)]),
  close(Stream).

% XUnit term generator
xunit_term_(Module,
  element(testsuite,
    [ name=Module, tests=NumTests,
      failures=NumFailures, errors=NumErrors,
      time=TestTime ],
    TestCaseTerms
  )) :-
  %%
  test_suite_begin(Module,T0),
  test_suite_end(Module,T1),
  TestTime is T1 - T0,
  %%
  findall(X0, test_case_begin(Module,X0,_), TestCases),
  length(TestCases,NumTests),
  %%
  findall(X1, (
    test_case_failure(Module,X1,Failure),
    xunit_is_failure_(Failure)
  ), Failures),
  length(Failures,NumFailures),
  %%
  findall(X2, (
    test_case_failure(Module,X2,Err),
    xunit_is_error_(Err)
  ), Errors),
  length(Errors,NumErrors),
  %%
  findall(TestTerm, (
    member(X3, TestCases),
    xunit_test_term_(Module,X3,TestTerm)
  ), TestCaseTerms).

xunit_test_term_(Module,TestCase,
  element(testcase,
    [ name=TestCase, time=TestTime ],
    FailureTerms
  )) :-
  %%
  test_case_begin(Module,TestCase,T0),
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
  element(failure, [ type=failed, message='goal failed' ], [])) :- !.
xunit_failure_term_(succeeded(_),
  element(failure, [ type=failed,
  message='goal succeeded but should have failed' ], [])) :- !.

xunit_failure_term_(Error,
  element(error, [ type=exception, message=Msg ], [])) :-
  atom(Error) -> Msg = Error ; term_to_atom(Error,Msg).
