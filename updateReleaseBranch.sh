#!/bin/bash


# list of packages 
RELEASED_PKGS="3rdparty comp_ehow comp_germandeli comp_temporal ias_prolog_addons json_prolog mod_srdl \
               rosdep.yaml semweb stack.xml thea CMakeLists.txt comp_cop comp_spatial ias_knowledge_base \
               ias_semantic_map mod_probcog mod_vis rosprolog srldb Makefile"

# svn URLs
SVN_TRUNK='http://code.in.tum.de/svn/knowrob/trunk'
SVN_RELEASE='http://code.in.tum.de/svn/knowrob/branches/release'
SVN_LATEST='http://code.in.tum.de/svn/knowrob/tags/latest'

# build svn arguments string
SVN_ARGS=""
for PKG in $RELEASED_PKGS
do
  SVN_ARGS="${SVN_ARGS} ${SVN_TRUNK}/${PKG}"
#   echo ${SVN_ARGS}
done

# update release branch
svn delete ${SVN_RELEASE} -m 'Updating release branch: deleting before copying new data'
svn mkdir ${SVN_RELEASE} -m 'Updating release branch: re-creating release branch'
svn copy ${SVN_ARGS} ${SVN_RELEASE} -m 'Updating release branch: copying new data'

# update latest tag
svn delete ${SVN_LATEST} -m 'Updating latest tag: deleting before copying new data'
svn mkdir ${SVN_LATEST} -m 'Updating latest tag: re-creating release branch'
svn copy ${SVN_ARGS} ${SVN_LATEST} -m 'Updating latest tag: copying new data'

