#!/usr/bin/python

import os

jars = []
for path, dirs, files in os.walk("lib"):
    for f in files:
        if f[-4:] == '.jar' and f != "srldb.jar":
            jars.append(os.path.join(os.path.abspath(path), f))

bin = os.path.abspath("bin")
if not os.path.exists(bin):
    os.mkdir(bin)
paths = []
for path, dirs, files in os.walk(os.path.abspath("src")):
    print path
    if len(filter(lambda x: x[-5:] == ".java", files)) > 0:
        paths.append(os.path.join(path, "*.java"))

print "\nbuilding... target folder: %s" % bin
os.system("javac -d %s -cp %s %s" % (bin, os.path.pathsep.join(jars), " ".join(paths)))
