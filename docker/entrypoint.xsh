#!/usr/bin/xonsh

import sys

$RAISE_SUBPROC_ERROR = True

if ($REFRASIN_UID is not None) and ($REFRASIN_UID != $ORIG_REFRASIN_UID):
    usermod -o -u @($REFRASIN_UID) refrasin
    print("Running now with UID", $(id -u refrasin))

if ($REFRASIN_GID is not None) and ($REFRASIN_GID != $ORIG_REFRASIN_GID):
    groupmod -o -g @($REFRASIN_GID) refrasin
    print("Running now with GID", $(id -g refrasin))

chown refrasin:refrasin .

gosu refrasin @($ARGS[1:])
