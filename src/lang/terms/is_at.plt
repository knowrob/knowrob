
:- begin_tests(lang_is_at).

:- use_module(library('db/tripledb'), [ tripledb_load/2 ]).
:- use_module(library('lang/query'), [ tell/1, ask/1 ]).

:- tripledb_load(
        'package://knowrob/owl/test/swrl.owl',
        [ graph(user),
          namespace(test_swrl,'http://knowrob.org/kb/swrl_test#')
        ]).

:- use_module('./is_at.pl').

test('no_pose(Fred)') :-
	\+ is_at(test_swrl:'Fred',_).

test('no_pose(Driver)') :-
	\+ is_at(test_swrl:'Driver',_).

test('tell_pose(Fred)') :-
	tell( during(
		is_at(
			'http://knowrob.org/kb/swrl_test#Fred',
			[world,[1,0,0],[0,0,0,1]]
		),
		[5,15])
	).

test('has_pose(Fred)') :-
	ask(during(is_at(test_swrl:'Fred',_), [9,12])).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- end_tests(lang_is_at).
