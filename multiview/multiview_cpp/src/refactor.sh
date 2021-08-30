#!/bin/bash

TMPD="$(mktemp -d /tmp/$(basename $0).XXXXXX)"
trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

do_search()
{
    N=100

    [ "$#" -eq 1 ] && TEXT="$1"
    [ "$#" -eq 2 ] && TEXT="$2" && N="$1"

    D="*"

    while [ "1" = "1" ] ; do
        fgrep -n "$TEXT" $D 2>/dev/null
        [ "$N" -eq "0" ] && exit 0
        N=$(expr $N - 1)
        D="$D/*"
    done
}

[ "$#" != "2" ] && echo "expected two arguments, aborting" && exit 1

SEARCH="$1"
REPLACE="$2"

do_search $SEARCH | awk -F':' '{ print $1 }' | sort | uniq | tee $TMPD/files

cat $TMPD/files | while read F ; do
    sed -i "s,$SEARCH,$REPLACE,g" "$F"
done

