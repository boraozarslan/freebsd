# $FreeBSD$

.include <src.opts.mk>

PROG=		loader.sym
INTERNALPROG=

BINDIR?=	/boot
INSTALLFLAGS=	-b
WARNS?=		1

# Architecture-specific loader code
SRCS=	start.S \
	main.c \
	conf.c \
	vers.c

SRCS+=	qcomm_console.c

# Always add MI sources
.PATH:		${.CURDIR}/../../common
SRCS+=		console.c \
		panic.c
CFLAGS+=	-I.
CFLAGS+=	-I${.CURDIR}/../../common
CFLAGS+=	-I${.CURDIR}/../../../contrib/libfdt

FILES=		loader.bin
FILESMODE_loader.bin= ${BINMODE}
CLEANFILES+=	vers.c loader.sym

CFLAGS+=	-fno-builtin -mgeneral-regs-only
CFLAGS+=	-ffreestanding

# FDT support
CFLAGS+=	-I${.CURDIR}/../../fdt
CFLAGS+=	-I${.CURDIR}/../../../contrib/libfdt/
LIBFDT=		${.OBJDIR}/../../fdt/libfdt.a

LDSCRIPT=	${.CURDIR}/ldscript.arm64
LDFLAGS=	-nostdlib
LDFLAGS+=	-static
LDFLAGS+=	-Wl,-T${LDSCRIPT}

# where to get libstand from
CFLAGS+=	-I${.CURDIR}/../../../../lib/libstand/

# clang doesn't understand %D as a specifier to printf
NO_WERROR.clang=

#DPADD=		${LIBSTAND}
DDADD=		${LIBFDT} ${.OBJDIR}/../../../../lib/libstand/libstand.a
#LDADD=		-lstand
LDADD=		${LIBFDT} ${.OBJDIR}/../../../../lib/libstand/libstand.a

NEWVERSWHAT=	"Linux ABI loader" ${MACHINE_CPUARCH}

vers.c:	${.CURDIR}/../../common/newvers.sh ${.CURDIR}/version
	sh ${.CURDIR}/../../common/newvers.sh ${.CURDIR}/version ${NEWVERSWHAT}

loader.sym:	${LDSCRIPT}

loader.bin: loader.sym
	if [ `${OBJDUMP} -t ${.ALLSRC} | fgrep '*UND*' | wc -l` != 0 ]; then \
		${OBJDUMP} -t ${.ALLSRC} | fgrep '*UND*'; \
		exit 1; \
	fi
	${OBJCOPY} -S -O binary loader.sym ${.TARGET}

loader-qcomm.img: loader.bin ${.OBJDIR}/../../../GENERIC/kernel
	/usr/local/bin/python ${.CURDIR}/mkbootimg \
	    --kernel ${.OBJDIR}/loader.bin \
	    --ramdisk ${.OBJDIR}/../../../GENERIC/kernel \
	    --dt /home/andrew/freebsd/arm64/qualcomm/dt.img \
	    --output loader-qcomm.img \
	    --pagesize 2048 \
	    --base 0x80000000

MAN=

.include <bsd.prog.mk>
