#!/bin/sh
# reformat.sh dir -> remove all ^M, single space *.h, gnu indent *.{ch}

if [[ ! -d "$1" ]]; then echo usage: reformat.sh dir; exit 1; fi

find $1 -type f -name '*.h' -print 
find $1 -type f -name '*.h' -print0 | xargs -0 sed -i 's/\t/ /g'
find $1 -type f -name '*.h' -print0 | xargs -0 sed -i 's/  */ /g'
find $1 -type f -name '*.h' -print0 | xargs -0 sed -i 's/\r//g'
find $1 -type f -name '*.h' -print0 | xargs -0 indent
find $1 -type f -name '*.c' -print 
find $1 -type f -name '*.c' -print0 | xargs -0 sed -i 's/\r//g'
find $1 -type f -name '*.c' -print0 | xargs -0 indent
