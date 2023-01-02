:- module(url,
        [ url_resolve/2
        ]).
/** <module> Resolving URLs to local filesystem.

@author Daniel Beßler
@license BSD
*/

%% url_resolve(+URL, -Resolved) is nondet.
%
% A multifile predicate used to support special URL schemas
% like ROS package URLs of the form "package://rospkg/....".
%
:- multifile url_resolve/2.
