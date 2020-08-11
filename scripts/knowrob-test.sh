#!/bin/sh

export KNOWROB_MONGO_DB_NAME="knowrobTestDb"
export KB_WHIPE_TRIPLE_STORE="true"

rosrun rosprolog rosprolog-test "$1"