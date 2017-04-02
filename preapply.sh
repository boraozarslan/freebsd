#!/bin/sh
if [ x"$INO64GIT" = x ]; then
    echo "set INO64GIT env var to point to the git mirror of ino64"
    exit 1
fi
xxx=$(svn st 2>&1)
if [ x"$xxx" \!= x ]; then
    echo "working directory is not clean, check with svn status"
    exit 1
fi
set -e
svn cp include/fts.h lib/libc/gen/fts-compat11.h
rm lib/libc/gen/fts-compat11.h
svn cp include/glob.h lib/libc/gen/glob-compat11.h
rm lib/libc/gen/glob-compat11.h
svn cp lib/libc/gen/fts.c lib/libc/gen/fts-compat11.c
rm lib/libc/gen/fts-compat11.c
svn cp lib/libc/gen/ftw.c lib/libc/gen/ftw-compat11.c
rm lib/libc/gen/ftw-compat11.c
svn cp lib/libc/gen/nftw.c lib/libc/gen/nftw-compat11.c
rm lib/libc/gen/nftw-compat11.c
svn cp lib/libc/gen/glob.c lib/libc/gen/glob-compat11.c
rm lib/libc/gen/glob-compat11.c
svn cp lib/libc/gen/getmntinfo.c lib/libc/gen/getmntinfo-compat11.c
rm lib/libc/gen/getmntinfo-compat11.c
svn cp lib/libc/gen/readdir.c lib/libc/gen/readdir-compat11.c
rm lib/libc/gen/readdir-compat11.c
svn cp lib/libc/gen/scandir.c lib/libc/gen/scandir-compat11.c
rm lib/libc/gen/scandir-compat11.c
(cd $INO64GIT && git diff origin/master ino64) | gpatch -p1
add_files="lib/libc/sys/getdents.c lib/libc/sys/lstat.c"
add_files="$add_files lib/libc/sys/mknod.c lib/libc/sys/stat.c"
add_files="$add_files lib/libc/gen/gen-compat.h lib/libc/gen/devname-compat11.c"
add_files="$add_files lib/libprocstat/libprocstat_compat.c"
svn add $add_files
svn propset svn:keywords 'FreeBSD=%H' $add_files
rm -f TODO.ino64 preapply.sh abi.sh ports.patch
xxx=$(svn st | awk '$1 ~ /^\?$/ {print $2}')
if [ x"$xxx" \!= x ]; then
    echo "unmanaged files were found after patching, check"
    exit 1
fi
echo "Looks good"
