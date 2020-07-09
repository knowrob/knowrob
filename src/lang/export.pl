:- module(lang_export,
    [ remember/1,
      memorize/1
    ]).
/** <module> Interface for dumping knowledge and restoring it.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('db/tripledb'),
    [ tripledb_import/1,
      tripledb_export/1
    ]).
:- use_module(library('db/obda'),
    [ obda_import/1,
      obda_export/1
    ]).
:- use_module(library('comm/notify'),
    [ notify/1 ]).

:- multifile remember_hook/1.
:- multifile memorize_hook/1.

%% remember(+Directory) is det.
%
% Restore memory previously stored into given directory.
%
% @param Directory filesystem path
%
remember(Directory) :-
  tripledb_import(Directory),
  obda_import(Directory),
  forall(remember_hook(Directory), true),
  forall(is_individual(I), notify(individual(I))).

%% memorize(+Directory) is det.
%
% Store knowledge into given directory.
%
% @param Directory filesystem path
%
memorize(Directory) :-
  tripledb_export(Directory),
  obda_export(Directory),
  forall(memorize_hook(Directory), true).

     /*******************************
     *          UNIT TESTS          *
     *******************************/

:- begin_tests('lang_export').

get_path(Path):-
  working_directory(X,X), string_concat(X, "test_lang_export", Path).

test('stores knowledge in test_lang_export directory', 
  [ fixme('tf_plugin not loaded properly'),
    setup(get_path(Path))
  ]) :-
  memorize(Path).

test('wipe db') :-
  tripledb_whipe.

test('restores knowledge from test_lang_export directory', 
  [ fixme('tf_plugin not loaded properly'),
  setup(get_path(Path)),
  cleanup(shell('cd $(rospack find knowrob); rm -rf test_lang_export'))
  ]) :-
  remember(Path).

:- end_tests('lang_export').
