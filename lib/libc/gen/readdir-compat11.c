/*
 * Copyright (c) 1983, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * from:
 * $FreeBSD$
 */

#if defined(LIBC_SCCS) && !defined(lint)
static char sccsid[] = "@(#)readdir.c	8.3 (Berkeley) 9/29/94";
#endif /* LIBC_SCCS and not lint */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "namespace.h"
#include <sys/param.h>
#define	_WANT_FREEBSD11_DIRENT
#include <dirent.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "un-namespace.h"

#include "libc_private.h"
#include "gen-private.h"
#include "telldir.h"

#include "gen-compat.h"

static int
freebsd11_cvtdirent(struct freebsd11_dirent *dstdp, struct dirent *srcdp)
{

	if (srcdp->d_namlen > sizeof(dstdp->d_name) - 1)
		return (ENAMETOOLONG);
	dstdp->d_type = srcdp->d_type;
	dstdp->d_namlen = srcdp->d_namlen;
	dstdp->d_fileno = srcdp->d_fileno;		/* truncate */
	dstdp->d_reclen = FREEBSD11_DIRSIZ(dstdp);
	bcopy(srcdp->d_name, dstdp->d_name, dstdp->d_namlen);
	bzero(dstdp->d_name + dstdp->d_namlen,
	    dstdp->d_reclen - offsetof(struct freebsd11_dirent, d_name) -
	    dstdp->d_namlen);
	return (0);
}

struct freebsd11_dirent *
freebsd11_readdir(DIR *dirp)
{
	static struct freebsd11_dirent *compatbuf;
	struct freebsd11_dirent *dstdp;
	struct dirent *dp;

	if (__isthreaded)
		_pthread_mutex_lock(&dirp->dd_lock);
again:
	dp = _readdir_unlocked(dirp, 1);
	if (dp != NULL) {
		if (compatbuf == NULL)
			compatbuf = malloc(sizeof(struct freebsd11_dirent));
		if (freebsd11_cvtdirent(compatbuf, dp) != 0)
			goto again;
		dstdp = compatbuf;
	} else
		dstdp = NULL;
	if (__isthreaded)
		_pthread_mutex_unlock(&dirp->dd_lock);

	return (dstdp);
}

int
freebsd11_readdir_r(DIR *dirp, struct freebsd11_dirent *entry,
    struct freebsd11_dirent **result)
{
	struct dirent xentry;
	struct dirent *xresult;
	int error;

again:
	error = readdir_r(dirp, &xentry, &xresult);

	if (error != 0)
		return (error);
	if (xresult != NULL) {
		if (freebsd11_cvtdirent(entry, &xentry) != 0)
			goto again;
		*result = entry;
	} else
		*result = NULL;
	return (0);
}

__sym_compat(readdir, freebsd11_readdir, FBSD_1.0);
__sym_compat(readdir_r, freebsd11_readdir_r, FBSD_1.0);
