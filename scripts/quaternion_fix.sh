#!/bin/bash

# The input argument is the path to some OWL file
OWL_FILE="$1"
# path to owl file
DIR=$(dirname "${OWL_FILE}")
# name of the owl file
FILE=$(basename "${OWL_FILE}")
# directory where this script is stored
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# the python script for conversion
PY_SCRIPT="${SCRIPT_DIR}/quaternion_fix.py"

cd $DIR
IFS=''
cat $FILE | while read line
do
   X=`echo $line | grep quaternion`
   if [ -z "${X}" ] ; then
       echo $line
   else
       echo `python "${PY_SCRIPT}" $line`
   fi
done

