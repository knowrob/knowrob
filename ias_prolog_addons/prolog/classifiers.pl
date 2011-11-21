/** <module> classifiers

  Interface between Prolog and the Weka and Mallet machine learning libraries.

  This module provides predicates for training, applying, loading and saving
  classifiers and clusterers. It provides access to all routines implemented
  in Weka (several classifiers and clusterers) and Mallet (esp. Conditional
  Random Fields).

  Copyright (C) 2010 by Jakob Engel, Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


@author Jakob Engel, Moritz Tenorth
@license GPL
*/

:- module(classifiers,
    [
      classifier_trained/4,
      clusterer_trained/4,
      crf_trained/4,
      classify_instances/3,
      classify_instances_return_complete/3,
      crf_label/3,
      classifier_saved/2,
      clusterer_saved/2,
      crf_saved/2
    ]).
:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('util')).


%% classifier_trained(- Classifier, + Type, + Options, + Trainingsdata) is nondet.
%
% Interface to the Weka machine learning library
%
% Trains a classifier on the provided training data and returns the classifier instance
%
% Example: classifier_trained(C, 'weka.classifiers.bayes.NaiveBayes', '', [['ClassA', 3, 5] , ['ClassB', 1, 9], ...]).
%
% @param Classifier     The resulting classifier instance
% @param Type           Class of the weka classifier, e.g. 'weka.classifiers.bayes.NaiveBayes'
% @param Options        Options that are passed to the weka classifier
% @param Trainingsdata  Data the classifier is to be learned from, List of lists with the classes as the first entries

classifier_trained(Classifier, Type, Options, Trainingsdata) :-
  prepare_data(Trainingsdata, T1),
  check_weka_data(T1),
  lists_to_arrays(T1, Arg),
  jpl_call('de.tum.in.fipm.kipm.util.ClassifierWrapper', 'createWrapper', [Type, Options, Arg], Classifier).



%% clusterer_trained(- Classifier, + Type, + Options, + Trainingsdata) is nondet.
%
% Interface to the Weka machine learning library
%
% Trains a clusterer on the provided training data and returns the clusterer instance
%
% Example: classifier_trained(C, 'weka.clusterers.SimpleKMeans', '', [[3, 5] , [1, 9], ...]).
%
% @param Clusterer      The resulting clusterer instance
% @param Type           Class of the weka clusterer, e.g. 'weka.clusterers.SimpleKMeans'
% @param Options        Options that are passed to the weka classifier
% @param Trainingsdata  Data the classifier is to be learned from, List of lists of data

clusterer_trained(Clusterer, Type, Options, Trainingsdata) :-
  prepare_data(Trainingsdata, T1),
  check_weka_data(T1),
  lists_to_arrays(T1, Arg),
  jpl_call('de.tum.in.fipm.kipm.util.ClustererWrapper', 'createWrapper', [Type, Options, Arg], Clusterer).



%% crf_trained(-CRF, +Options, +Trainingsdata, +Labels) is nondet.
%
% Interface to the Mallet machine learning library
%
% Trains a Conditional Random Fields (CRF) classifier on the provided training data and returns the CRF instance
%
% @param CRF              The resulting classifier instance
% @param Options          Options that are passed to the CRF class
% @param Trainingsdata    Data the classifier is to be learned from
% @param Labels           List of labels assigned to the data instances

crf_trained(CRF, Options, Trainingsdata, Labels) :-
  prepare_data(Trainingsdata, T1),
  prepare_data(Labels, L1),
  lists_to_arrays(T1, TArr),
  lists_to_arrays(L1, LArr),
  jpl_call('de.tum.in.fipm.kipm.util.MalletCRFWrapper', 'createWrapper', [Options, TArr, LArr], CRF).



%% classify_instances(+ Classifier, + Instances, - Classified) is nondet.
%
% Interface to the Weka machine learning library
%
% Classifies a list of instances, using a previously trained Clusterer / Classifier.
%
% Returns a list of the assigned classes in Classified.
%
% Example: classify_instances(C, [[3,b],[9,a]], R).
%          -> R = [classA,classB]
%
% @param Classifier   Classifier instance that is to be used for classification
% @param Instances    Data to be classified
% @param Classified   Resulting class/cluster assignment: [classA,classB]

classify_instances(Classifier, Instances, Classified) :-
  prepare_data(Instances, I1),
  check_weka_data(I1),
  lists_to_arrays(I1, InstancesArr),
  jpl_call(Classifier, classify, [InstancesArr], ClassifiedArr),
  arrays_to_lists(ClassifiedArr, Classified).



%% classify_instances_return_complete(+ Classifier, + Instances, - Classified) is nondet.
%
% Interface to the Weka machine learning library
%
% Similar to classify_instances, but returns the whole data with the class label added as first attribute.
%
% Example: classify_instances(C, [[3,b],[9,a]], R).
%          -> R = [[classA,3,b],[classB,9,a]]
%
% @param Classifier   Classifier instance that is to be used for classification
% @param Instances    Data to be classified
% @param Classified   Resulting class/cluster assignment in conjunction with the original data: [[classA,3,b],[classB,9,a]]

classify_instances_return_complete(Classifier, Instances, Classified) :-
  classify_instances(Classifier, Instances, ClTmp),
  merge_classify_results(Instances, ClTmp, Classified).



%% crf_label(+CRF, +Trainingsdata, -Labels) is nondet.
%
% Interface to the Mallet machine learning library
%
% Labels data using a given CRF
%
% @param CRF        CRF instance that is to be used for classification
% @param Testdata   Data to be labeled
% @param Labels     Inferred list of labels

crf_label(CRF, Testdata, Labels) :-
  prepare_data(Testdata, T1),
  lists_to_arrays(T1, TArr),
  jpl_call(CRF, label, [TArr], LabelsArr),
  arrays_to_lists(LabelsArr, Labels).


%% classifier_saved(?File,?Classifier) is nondet.
%
% Interface to the Weka machine learning library
%
% Loads / saves a classifier that has been created via classifier_trained to a specified file.
% Loading: classifier_saved(+File,-Classifier)
% Saving:  classifier_saved(+File,+Classifier)
%
% @param File       File name the classifier is to be saved to/loaded from
% @param Classifier The classifier instance to be saved or loaded

classifier_saved(File, Classifier) :-
  (var(Classifier), print('Loading Classifier...'), jpl_call('de.tum.in.fipm.kipm.util.ClassifierWrapper','loadFromFile',[File],Classifier)) ;
  (nonvar(Classifier), print('Saving Classifier...'),jpl_call(Classifier,'saveToFile',[File],_)).



%% clusterer_saved(?File,?Clusterer) is nondet.
%
% Interface to the Weka machine learning library
%
% Loads / saves a clusterer that has been created via clusterer_trained to a specified file.
% Loading: clusterer_trained(+File,-Clusterer)
% Saving:  clusterer_trained(+File,+Clusterer)
%
% @param File       File name the clusterer is to be saved to/loaded from
% @param Classifier The clusterer instance to be saved or loaded

  clusterer_saved(File, Clusterer) :-
    (var(Clusterer), print('Loading Clusterer...'), jpl_call('de.tum.in.fipm.kipm.util.ClustererWrapper','loadFromFile',[File],Clusterer)) ;
    (nonvar(Clusterer), print('Saving Clusterer...'),jpl_call(Clusterer,'saveToFile',[File],_)).



%% crf_saved(?File,?CRF) is nondet.
%
% Interface to the Mallet machine learning library
%
% Loads / saves a crf that has been created via crf_trained to a specified file.
% Loading: crf_saved(+File,-CRF)
% Saving:  crf_saved(+File,+CRF)
%
% @param File  File name the classifier is to be saved to/loaded from
% @param CRF   The classifier instance to be saved or loaded

  crf_saved(File, CRF) :-
    (var(CRF), print('Loading CRF...'), jpl_call('de.tum.in.fipm.kipm.util.MalletCRFWrapper','loadFromFile',[File],CRF)) ;
    (nonvar(CRF), print('Saving CRF...'),jpl_call(CRF,'saveToFile',[File],_)).




% --------------------------- internal helper functions  --------------------------------------

% classifier_trained(C,'weka.classifiers.bayes.NaiveBayes','',[[a,1,10],[a,2,11],[b,-4,3],[b,-3,2],[a,3,11],[a,4,10],[a,2,12],[b,-5,2],[b,-2,3]]), classifier_saved('~/work/owl/classifier.bin',C), classify_instances(C,[[1,10],[2,9],[-2,2],[-4,6]],R).
% classifier_saved('~/work/owl/classifier.bin',C), classify_instances(C,[[1,10],[2,9],[-2,2],[-4,6]],R).

crf_test_data(D,L) :- D=[[1,1,1,3,3,3,1,1],
                         [1,1,2,2,2,2,1,1,1,1],
                         [1,2,2,2,2,1,1,1],
                         [1,1,1,3,1,1,1,1],
                         [1,3,3,3,3,3,1],
                         [1,2,2,2,2,1,1],
                         [1,2,1],
                         [1,2,2,2,1]],
                      L=[[a,a,a,c,c,c,d,d],
                         [a,a,b,b,b,b,d,d,d,d],
                         [a,b,b,b,b,d,d,d],
                         [a,a,a,c,d,d,d,d],
                         [a,c,c,c,c,c,d],
                         [a,b,b,b,b,d,d],
                         [a,b,d],
                         [a,b,b,b,d]].


% prepare_data(+Data, -Result)
% takes (multiple) lists, and
% - replaces every numeric values with the corresponding string.
% - removes literal(type(_,Value))
%
  prepare_data(X,Y) :- number(X), string_to_atom(X,Y), !.
  prepare_data(X,X) :- atom(X), !.
  prepare_data(literal(type(_,X)),Y) :- prepare_data(X,Y), !.
  prepare_data([],[]).
  prepare_data([A1|ARest],[B1|BRest]) :- prepare_data(A1,B1), prepare_data(ARest, BRest).


% check_weka_data(+Data)
% succeeds if Data is a non-empty List of equally long, non-empty lists of atoms.
%
  check_weka_data([F | Rest]) :- length(F, X), >(X,0), atom_list_length(F, X), check_all(Rest, X).
  atom_list_length([],0).
  atom_list_length([H|T],L) :- atom(H), >(L,0), LN is L-1, atom_list_length(T,LN).
  check_all([],_).
  check_all([H|T],L) :- atom_list_length(H,L), check_all(T,L).


% merge_classify_results(+ Data, +FirstElements, -Result)
%
% appends elements of FirstElement to Data.
merge_classify_results([D1 | Data], [FE1 | FirstElements], [[FE1 | D1] | Result]) :-
  merge_classify_results(Data, FirstElements, Result).

merge_classify_results([], [], []).

