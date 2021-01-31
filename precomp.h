#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
#ifndef __LINUX_MUTEX_H
#define __LINUX_MUTEX_H

#include <asm/semaphore.h>

#define mutex semaphore
#define DEFINE_MUTEX(foo) DECLARE_MUTEX(foo)
#define mutex_init(foo) init_MUTEX(foo)
#define mutex_lock(foo) down(foo)
#define mutex_lock_interruptible(foo) down_interruptible(foo)
/* this function follows the spin_trylock() convention, so        *
 * it is negated to the down_trylock() return values! Be careful  */
#define mutex_trylock(foo) !down_trylock(foo)
#define mutex_unlock(foo) custom_up(foo)

/*
 * Note! This is subtle. We jump to wake people up only if
 * the semaphore was negative (== somebody was waiting on it).
 * The default case (no contention) will result in NO
 * jumps for both down() and up().
 */
static inline void custom_up(struct semaphore * sem)
{
	__asm__ __volatile__(
		"# atomic up operation\n\t"
		LOCK "incl %0\n\t"     /* ++sem->count */
		"jle 2f\n"
		"1:\n"
		LOCK_SECTION_START("")
		"2:\tcall __up_wakeup\n\t"
		"jmp 1b\n"
		LOCK_SECTION_END
		".subsection 0\n"
		:"=m" (sem->count)
		:"c" (sem)
		:"memory");
}

#endif /* __LINUX_MUTEX_H */
#endif

#ifndef __ADV_PRECOMP_H
#define __ADV_PRECOMP_H

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
#define SERIAL_IO_AU      4
#define SERIAL_IO_TSI     5
#define SERIAL_IO_MEM32BE 6
#define SERIAL_IO_MEM16   7

#define UPIO_AU				(SERIAL_IO_AU)		/* Au1x00 and RT288x type IO */
#define UPIO_TSI			(SERIAL_IO_TSI)		/* Tsi108/109 type IO */
#define UPIO_MEM32BE		(SERIAL_IO_MEM32BE)	/* 32b big endian */
#define UPIO_MEM16			(SERIAL_IO_MEM16)	/* 16b little endian */

#define UART_MCR_CLKSEL	0x80 	/* Divide clock by 4 (TI16C752, EFR[4]=1) */

struct uart_match {
    struct uart_port *port;
    struct uart_driver *driver;
};

static inline int
uart_match_port(struct uart_port *port1, struct uart_port *port2)
{
    if (port1->iotype != port2->iotype)
        return 0;

    switch (port1->iotype) {
    case UPIO_PORT:
        return (port1->iobase == port2->iobase);
    case UPIO_HUB6:
        return (port1->iobase == port2->iobase) &&
               (port1->hub6   == port2->hub6);
    case UPIO_MEM:
    case UPIO_MEM16:
    case UPIO_MEM32:
    case UPIO_MEM32BE:
    case UPIO_AU:
    case UPIO_TSI:
        return (port1->mapbase == port2->mapbase);
    }
    return 0;
}

static inline void
uart_insert_char(struct uart_port *port, unsigned int status,
		 unsigned int overrun, unsigned int ch, unsigned int flag)
{
	struct tty_struct *tty = port->info->tty;

	if ((status & port->ignore_status_mask & ~overrun) == 0)
		tty_insert_flip_char(tty, ch, flag);

	/*
	 * Overrun is special.  Since it's reported immediately,
	 * it doesn't affect the current character.
	 */
	if (status & ~port->ignore_status_mask & overrun)
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
}

#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
#define DIV_ROUND_CLOSEST(x, divisor)(			\
{							\
	typeof(divisor) __divisor = divisor;		\
	(((x) + ((__divisor) / 2)) / (__divisor));	\
}							\
)
#endif

/*
 * older kernel as 2.6.18 not define UART_LSR_BRK_ERROR_BITS
 * 0x1E means BI, FE, PE, OE bits
 */
#ifndef UART_LSR_BRK_ERROR_BITS
#define UART_LSR_BRK_ERROR_BITS         0x1E
#endif

#endif /* __ADV_PRECOMP_H */
