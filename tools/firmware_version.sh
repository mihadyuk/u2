#!/bin/bash

OUT=firmware_version.h
rm -f $OUT

HASH="static const char *commit_hash = \"\\t"
HASH="$HASH`git --no-pager log -1 | grep commit`\";"
echo $HASH > $OUT

DATE="static const char *commit_date = \"\\t"
DATE="$DATE`git --no-pager log -1 | grep Date`\";"
echo $DATE >> $OUT

