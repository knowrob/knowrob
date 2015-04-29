#!/bin/bash
# Author: Daniel Beßler

SCRIPT=`readlink -f "$0"`
DIR=`dirname $SCRIPT`

OLD_DST=$DIR/knowrob_vis/src/main/java/org/knowrob/vis/collada
if [ -d "$OLD_DST" ]; then
  echo "Removing $OLD_DST"
  rm -rf $OLD_DST
fi

DST=$DIR/knowrob_vis/src/main/java/org/knowrob/vis/collada_1_4_1
if [ ! -d "$DST" ]; then
  echo "Generating COLLADA JAVA classes (please ignore GConf errors)...."
  xjc $DIR/schema/collada/collada-schema-1.4.1.xsd \
      -d $DIR/knowrob_vis/src/main/java \
      -p org.knowrob.vis.collada_1_4_1 \
      -extension $DIR/schema/simpleMode.xsd
  echo "COLLADA JAVA classes generated."
fi

