#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "bootstrap.h"

/* Status */
#define	UART_DM_SR		0x008
#define	 SR_RX_FULL		(1 << 1)
#define	 SR_TX_EMPTY		(1 << 3)

/* TX Character Count */
#define	UART_DB_TC_CHARS	0x040

/* TX FIFO */
#define	UART_DM_TF		0x0100

static uint32_t *ubase = (uint32_t *)0x78b0000;

static uint32_t
uart_getreg(uint32_t *bas, int off)
{

	return *((volatile uint32_t *)((uintptr_t)bas + off));
}

static void
uart_setreg(uint32_t *bas, int off, uint8_t val)
{

	*((volatile uint32_t *)((uintptr_t)bas + off)) = (uint32_t)val;
}

static void
qcomm_cons_probe(struct console *cp)
{
	cp->c_flags |= C_PRESENTIN | C_PRESENTOUT;
}

static int
qcomm_cons_init(int arg)
{

	return (0);
}

void
qcomm_cons_putchar(int ch)
{
	int limit;

	limit = 20;
	while ((uart_getreg(ubase, UART_DM_SR) & SR_TX_EMPTY) == 0 && --limit)
		delay(5);

	/* Write 1 character */
	uart_setreg(ubase, UART_DB_TC_CHARS, 1);
	/* Write sait character to the FIFO */
	uart_setreg(ubase, UART_DM_TF, ch);
}

static int
qcomm_cons_getchar(void)
{
#if 0
	int c, limit;

	c = 0;
	while ((uart_getreg(ubase, SR) & SR_RXREADY) == 0)
		delay(1000);

	c = uart_getreg(ubase, DATA);

	return c;
#endif
	return (int)'A';
}

static int
qcomm_cons_poll(void)
{

	return (0);
	//return ((uart_getreg(ubase, SR) & SR_RXREADY) != 0 ? 1 : 0);
}

struct console qcomm_console = {
	"qcomm",
	"Qualcomm console",
	0,
	qcomm_cons_probe,
	qcomm_cons_init,
	qcomm_cons_putchar,
	qcomm_cons_getchar,
	qcomm_cons_poll
};
