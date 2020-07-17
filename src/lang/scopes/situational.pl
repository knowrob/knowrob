%:- module(situational_scope,
    %[ within(r,r),
      %op(1000, yfx, within)
    %]).

% idea: "holds(...) within Event"
%       "Object hasRole Role since T within Situation"
%       "remember when I did ... does xyz hold?"
%       --> make role foo more convinient with sit scope
%
% idea: situation scope changes determine:
%         - set of objects of consideration
%         - the current object pose foo
%         - robot also refers to sit in live case
%
% XXX: how to handle "remember all situations where ..."?
%
