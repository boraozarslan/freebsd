/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2000, 2001 David O'Brien
 * Copyright (c) 1996 Søren Schmidt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/stat.h>
#include <sys/elf_common.h>
#include <err.h>
#include <fcntl.h>
#include <gelf.h>
#include <getopt.h>
#include <libelf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "_elftc.h"

__FBSDID("$FreeBSD$");

static int convert_to_feature_val(char*, u_int32_t*);
static int edit_file_features(Elf *, int, int, char*);
static int get_file_features(Elf *, int, int, u_int32_t *, u_int64_t *);
static int get_note_header(Elf *, int, GElf_Phdr *);
static void print_features(void);
static int print_file_features(Elf *, int, int);
static void usage(void);

struct ControlFeatures {
	const char *alias;
	unsigned long featureVal;
	const char *desc;
};

// NT_FREEBSD_FEATURE_CTL
static struct ControlFeatures featurelist[] = {
	{ "aslr",	NT_FREEBSD_FCTL_ASLR_DISABLE,			"Disable ASLR" },
	{ "wx", 	NT_FREEBSD_FCTL_WX_ENABLE,			"Enable W^X" },
	{ "maxprot", 	NT_FREEBSD_FCTL_IMPLIED_MAX_PROT_ENABLE,	"Enable implied max prot" }
};

static struct option controlelf_longopts[] = {
	{ "help",	no_argument,	NULL,   'h' },
	{ NULL,		0,		NULL,	0   }
};

int
main(int argc, char **argv)
{
	GElf_Ehdr ehdr;
	Elf *elf;
	Elf_Kind kind;
	int ch, fd, listed, editfeatures, retval;
	char *features;

	listed = 0;
	editfeatures = 0;
	retval = 0;

	if (elf_version(EV_CURRENT) == EV_NONE)
		errx(EXIT_FAILURE, "elf_version error");

	while ((ch = getopt_long(argc, argv, "hle:", controlelf_longopts,
		NULL)) != -1)
		switch (ch) {
		case 'l':
			print_features();
			listed = 1;
			break;
		case 'e':
			features = optarg;
			editfeatures = 1;
			break;
		case 'h':
			usage();
			break;
		default:
			usage();
	}
	argc -= optind;
	argv += optind;
	if (!argc) {
		if (listed)
			exit(0);
		else {
			warnx("no file(s) specified");
			usage();
		}
	}

	while (argc) {
		elf = NULL;

		if ((fd = open(argv[0], editfeatures ? O_RDWR :
		    O_RDONLY, 0)) < 0) {
			warn("error opening file %s", argv[0]);
			retval = 1;
			goto fail;
		}

		if ((elf = elf_begin(fd, ELF_C_READ, NULL)) == NULL) {
			warnx("elf_begin failed: %s", elf_errmsg(-1));
			retval = 1;
			goto fail;
		}

		if ((kind = elf_kind(elf)) != ELF_K_ELF) {
			if (kind == ELF_K_AR)
				warnx("file '%s' is an archive.", argv[0]);
			else
				warnx("file '%s' is not an ELF file.",
				    argv[0]);
			retval = 1;
			goto fail;
		}

		if (gelf_getehdr(elf, &ehdr) == NULL) {
			warnx("gelf_getehdr: %s", elf_errmsg(-1));
			retval = 1;
			goto fail;
		}

		if (!editfeatures) {
			printf("File '%s' features:\n", argv[0]);
			if (print_file_features(elf, ehdr.e_phnum, fd) == 0) {
				retval = 1;
				goto fail;
			}
		} else if (edit_file_features(elf, ehdr.e_phnum, fd, features) == 0) {
			retval = 1;
			goto fail;
		}
fail:

		if (elf)
			elf_end(elf);

		if (fd >= 0 && close(fd) == -1) {
			warnx("%s: close error", argv[0]);
			retval = 1;
		}

		argc--;
		argv++;
	}

	return (retval);
}

#define	USAGE_MESSAGE	"\
Usage: %s [options] file...\n\
  Set or display the control features for an ELF object.\n\n\
  Supported options are:\n\
  -l                        List known CTL features.\n\
  -e [+-=]feature,list      Edit features from a comma separated list.\n\
  -h | --help               Print a usage message and exit.\n"

static void
usage(void)
{
	fprintf(stderr, USAGE_MESSAGE, ELFTC_GETPROGNAME());
	exit(1);
}

static int
get_note_header(Elf *elf, int phcount, GElf_Phdr *phdr)
{
	int i;

	for (i = 0; i < phcount; ++i) {
		if (phdr != gelf_getphdr(elf, i, phdr)) {
			warnx("gelf_getphdr failed: %s", elf_errmsg(-1));
			return 0;
		}

		if (phdr->p_type == PT_NOTE)
			return 1;
	}
	return 0;
}

static int
convert_to_feature_val(char* feature_str, u_int32_t* feature_val)
{
	char *feature_input, *feature;
	int i, len, add = 0, set = 0;
	u_int32_t input = 0;

	if (feature_str[0] == '+')
		add = 1;
	else if (feature_str[0] == '=')
		set = 1;
	else if (feature_str[0] != '-') {
		warnx("'%c' is not an operator. Use instead '+', '-', '='.",
		    feature_str[0]);
		return 0;
	}

	feature_input = feature_str + 1;
	len = sizeof(featurelist) / sizeof(featurelist[0]);
	while ((feature = strsep(&feature_input, ",")) != NULL) {
		for (i = 0; i < len; ++i) {
			if (strcmp(featurelist[i].alias, feature) == 0) {
				input |= featurelist[i].featureVal;
				break;
			}
		}
		if (i == len) {
			warnx("%s is not a valid feature.", feature);
			return 0;
		}
	}

	if (add) {
		*feature_val |= input;
	} else if (set) {
		*feature_val = input;
	} else {
		*feature_val -= ((*feature_val) & input);
	}
	return 1;
}

static int
edit_file_features(Elf *elf, int phcount, int fd, char *val)
{
	u_int32_t features;
	u_int64_t off;

	if (get_file_features(elf, phcount, fd, &features, &off) == 0) {
		warnx("No CTL features note on the file.\n");
		return 0;
	}

	if (convert_to_feature_val(val, &features) == 0) return 0;

	lseek(fd, off, SEEK_SET);
	write(fd, &features, sizeof(u_int32_t));
	return 1;
}

static void
print_features(void)
{
	size_t i;

	printf("Known features are:\n");
	for (i = 0; i < sizeof(featurelist)/sizeof(featurelist[0]); ++i)
		printf("%s\t\t %s\n", featurelist[i].alias,
		    featurelist[i].desc);
}

static int
print_file_features(Elf *elf, int phcount, int fd) {
	u_int32_t features;
	unsigned long i;

	if (get_file_features(elf, phcount, fd, &features, NULL) == 0) {
		warnx("No features note in the file.\n");
		return 0;
	}

	for (i = 0; i < sizeof(featurelist)/sizeof(featurelist[0]); ++i) {
		printf("%s\t\t'%s' is ", featurelist[i].alias,
		    featurelist[i].desc);

		if ((featurelist[i].featureVal & features) == 0)
			printf("un");

		printf("set.\n");
	}
	return 1;
}

static int
get_file_features(Elf *elf, int phcount, int fd, u_int32_t *features, u_int64_t *off)
{
	GElf_Phdr phdr;
	Elf_Note note;
	unsigned long read_total;
	int namesz, descsz;
	char *name, *desc;

	if (get_note_header(elf, phcount, &phdr) == 0) {
		warnx("Couldn't find a note header");
		return 0;
	}

	lseek(fd, phdr.p_offset, SEEK_SET);
	read_total = 0;

	while (read_total < phdr.p_filesz) {
		read(fd, &note, sizeof(note));
		read_total += sizeof(note);

		/*
		 * XXX: Name and descriptor are 4 byte aligned, however,
		 * 	the size given doesn't include the padding.
		 */
		namesz = ((note.n_namesz + 3) / 4) * 4;
		name = calloc(namesz, sizeof(char));
		if (name == NULL) {
			warnx("Calloc failed.\n");
			return 0;
		}
		descsz = ((note.n_descsz + 3) / 4) * 4;
		read(fd, name, namesz);
		read_total += (unsigned long)namesz;

		if (strncmp("FreeBSD", name, 7) != 0 ||
		    note.n_type != NT_FREEBSD_FEATURE_CTL) {
			desc = calloc(descsz, sizeof(char));
			if (desc == NULL) {
				warnx("Calloc failed.\n");
				free(name);
				return 0;
			}
			read(fd, desc, descsz);
			read_total += (unsigned long) descsz;
			free(name);
			free(desc);
			continue;
		}

		if (note.n_descsz < sizeof(u_int32_t)) {
			warnx("Feature descriptor can't be less than 4 bytes");
			free(name);
			return 0;
		}

		/*
		 * XXX: For now we look at only 4 bytes of the descriptor
		 * 	should respect descsz.
		 */
		read(fd, features, sizeof(u_int32_t));
		if (off != NULL) *off = phdr.p_offset + read_total;
		free(name);
		return 1;
	}
	return 0;
}
