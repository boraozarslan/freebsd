/*-
 * Copyright (c) 2014 Andrew Turner
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <machine/asm.h>

	.text
ENTRY(_start)
	/* Save the boot params to the stack */
	stp	x0, x1, [sp, #-16]!

	/* Self relocate */
	adr	x0, ImageBase
	adr	x1, _DYNAMIC
	bl	self_reloc

	/* Hint where to look for the API signature */
	ldr	x8, =uboot_address
	mov	x9, sp
	str	x9, [x8]

	/* Save x18, it's used by U-Boot */
	ldr	x8, =saved_regs
	str	x18, [x8, #0]

	ldp	x0, x1, [sp], #16

	/* Tail-recurse to main */
	b	main
END(_start)

/*
 * syscall()
 */
ENTRY(syscall)
	str	x18, [sp, #-32]!
	stp	x29, x30, [sp, #16]

	/* Restore the U-Boot x18 */
	ldr	x9, =saved_regs
	ldr	x18, [x9]

	ldr	lr, =return_from_syscall
	ldr	x9, =syscall_ptr
	ldr	x9, [x9]
	br	x9

return_from_syscall:
	ldp	x29, x30, [sp, #16]
	ldr	x18, [sp], #32
	ret
END(syscall)

	.data
	.align	8

	.globl	syscall_ptr
syscall_ptr:
	.quad	0

	.globl	uboot_address
uboot_address:
	.quad	0

saved_regs:
	.quad	0	/* U-Boot x18 */
