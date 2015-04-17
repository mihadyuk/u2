#!/bin/bash

OUT=firmware_version.h
rm -f $OUT

HASH="static const char *commit_hash = \""
HASH="$HASH`git --no-pager log -1 | grep commit`"
HASH="$HASH\r\n\";"
echo $HASH > $OUT

DATE="static const char *commit_date = \""
DATE="$DATE`git --no-pager log -1 | grep Date`"
DATE="$DATE\r\n\";"
echo $DATE >> $OUT

