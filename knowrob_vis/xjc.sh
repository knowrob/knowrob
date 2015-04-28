#!/bin/bash
# Author: Daniel Beßler

SCRIPT=`readlink -f "$0"`
DIR=`dirname $SCRIPT`
DST=$DIR/knowrob_vis/src/main/java/org/knowrob/vis/collada

if [ ! -d "$DST" ]; then
  echo "Generating COLLADA JAVA classes (please ignore GConf errors)...."
  xjc $DIR/schema/collada/collada-schema-1.4.1.xsd \
      -d $DIR/knowrob_vis/src/main/java \
      -p org.knowrob.vis.collada \
      -extension $DIR/schema/simpleMode.xsd
  echo "COLLADA JAVA classes generated."
fi

