/*****************************************************************************/
/*
*      adv17v35x.c  -- Advantech multiport serial driver.
*
*      Copyright (C) 2016 Advantech Corporation.
*
*      Based on Linux 2.6.37 Kernel's  drivers/serial/8250.c and /8250_pci.c
*
*      This program is free software; you can redistribute it and/or modify
*      it under the terms of the GNU General Public License as published by
*      the Free Software Foundation; either version 2 of the License, or
*      (at your option) any later version.
*
*      This program is distributed in the hope that it will be useful,
*      but WITHOUT ANY WARRANTY; without even the implied warranty of
*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*      GNU General Public License for more details.
*
*      You should have received a copy of the GNU General Public License
*      along with this program; if not, write to the Free Software
*      Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
*
*	   Multiport Serial Driver for EXAR's PCI Family of UARTs (XR17V358/354/352)
*
*       ChangeLog:
*	   for			: LINUX 2.6.9 and newer (Tested on various kernel versions from 2.6.9 to 4.2.x)
*	   date			: Mar 2017
*	   version		: 5.0.3.0
*
*	Check Release Notes for information on what has changed in the new version.
*
*/

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/serial.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include "linux/version.h"
#include "precomp.h"

#define _INLINE_ inline

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0)
#define __devinitdata
#define __devinit
#define __devexit
#define __devexit_p
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
struct serial_uart_config {
	char	*name;
	int	dfl_xmit_fifo_size;
	int	flags;
};
#endif

//void tty_flip_buffer_push(struct tty_port *port);

/*
 * Definitions for PCI support.
 */

#define ADVANTECH_EXAR_VER	        "5.0.3.0"
#define ADVANTECH_EXAR_DATE	        "2017/03/06"
#define ADVANTECH_EXAR_FILE_VER     "5.0.3.0"

#define ADVANTECH_VID	            0x13FE

#define PCIE_1602		            0x000E
#define PCIE_1604		            0x000D
#define PCIE_1610		            0x000C
#define PCIE_1612		            0x000B
#define PCIE_1620		            0x000A
#define PCIE_1622		            0x0009

// support for beijing's cards
#define PCIE_0014					0x0014
#define PCIE_0015					0x0015
#define PCIE_0016					0x0016

#define PCM_3612I                   0x001F
#define PCI_1602                    0x0018
#define PCI_1604                    0x0019
#define PCI_1610                    0x001A
#define PCI_1612                    0x001B
#define PCI_1620                    0x001C
#define PCI_1622                    0x001D


#define FL_BASE_MASK		        0x0007
#define FL_BASE0		            0x0000
#define FL_GET_BASE(x)		        (x & FL_BASE_MASK)

#define NR_PORTS	                256

#define XR_MAJOR                    0
#define XR_MINOR                    0

/*
 * The special register set for XR17V35x UARTs.
 */
#define XR_17v35x_UART_RHR			0 
#define	XR_17V35X_UART_DLD	        2
#define	XR_17V35X_UART_MSR	        6
#define XR_17V35X_EXTENDED_FCTR		8
#define XR_17V35X_EXTENDED_EFR		9
#define XR_17V35X_TXFIFO_CNT		10
#define XR_17V35X_EXTENDED_TXTRG	10
#define XR_17V35X_RXFIFO_CNT		11
#define XR_17V35X_EXTENDED_RXTRG	11
#define XR_17V35X_UART_XOFF2        	13 
#define XR_17V35X_UART_XOFF1 		0xC0
#define XR_17V35X_UART_XON1		0xE0
#define XR_17V35X_FCTR_RTS_8DELAY	0x03
#define XR_17V35X_FCTR_TRGD		192
#define XR_17V35x_FCTR_RS485	        0x20

#define XR_17V35x_MPIOLVL_7_0       0x90
#define XR_17V35x_MPIO3T_7_0        0x91
#define XR_17V35x_MPIOSEL_7_0       0x93
#define XR_17V35x_MPIOLVL_15_8       0x96
#define XR_17V35x_MPIO3T_15_8        0x97
#define XR_17V35x_MPIOSEL_15_8       0x99


// Set this parameter to 1 to enabled internal loopback
#define ENABLE_INTERNAL_LOOPBACK      0

#define UART_17V35X_RX_OFFSET		0x100
#define UART_17V35X_TX_OFFSET 		0x100

#define	XR_17V35X_IER_RTSDTR	        0x40
#define XR_17V35X_IER_CTSDSR	        0x80

#define XR_17V35X_8XMODE	        0x88
#define XR_17V35X_4XMODE	        0x89

#define XR_17V35X_MPIOLVL_L		    0x90
#define XR_17V35X_MPIOLVL_H		    0x96

#define XR_17V35X_MPIOSEL_L		    0x93
#define XR_17V35X_MPIOSEL_H		    0x99

#define DIVISOR_CHANGED   0
#define XR_17V35X_RESET             0x8A

#define PCI_NUM_BAR_RESOURCES	6


/*
 * serial mode, RS232 RS422 Master  RS485/RS422 Slave
 */
#define SERIAL_MODE_UNKNOWN         0
#define SERIAL_MODE_RS232           1
#define SERIAL_MODE_RS422M          2
#define SERIAL_MODE_RS485ORRS422S   3

#define PASS_LIMIT					512

/*
 * macro for calcaute divide of baud
 */
#define TRUNC(X_Decuple)		            ( (X_Decuple)/10 )
#define ROUND(X_Decuple)		            ( ((X_Decuple)%10)>=5 ? (X_Decuple)/10+1 : (X_Decuple)/10 )		

#define 	ADVANTECH_EXAR_MAIGC            'x'
#define		ADV_READ_REG      				_IO(ADVANTECH_EXAR_MAIGC, 1)
#define 	ADV_WRITE_REG     				_IO(ADVANTECH_EXAR_MAIGC, 2)
#define 	ADV_GET_SERIAL_MODE				_IO(ADVANTECH_EXAR_MAIGC, 3)
#define 	ADV_SET_TURN_AROUND_DELAY		_IO(ADVANTECH_EXAR_MAIGC, 4)

/* For SPI bus, to get Board ID */
#define	REGB			0x8E
#define	EECK_BIT		0x10
#define	EECS_BIT		0x20
#define	EEDI_BIT		0x40
#define	EEDO_BIT		0x80

#define	GPIOA				0x12
#define	SPI_READ_CMD		0x41

/* For fixed number */
#define	MAX_STRING_LEN		64
#define	MAX_CARD_SUPPORT	16

/* DTR/DSR flow control support */
#ifndef CDTRDSR
#define CDTRDSR				004000000000		/* For DTR/DSR flow control */
#endif

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

struct serial_private {
	struct pci_dev		*dev;
	unsigned int		nr;
	void __iomem		*remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk	*quirk;
	int    uart_index[NR_PORTS];
	int			line[0];
};

struct pciserial_board {
	unsigned int flags;
	unsigned int num_ports;
	unsigned int base_baud;
	unsigned int uart_offset;
	unsigned int reg_shift;
	unsigned int first_offset;
};

/*
 * init function returns:
 *  > 0 - number of ports
 *  = 0 - use board->num_ports
 *  < 0 - error
 */
struct pci_serial_quirk {
	u32	vendor;
	u32	device;
	u32	subvendor;
	u32	subdevice;
	int	(*init)(struct pci_dev *dev);
	int	(*setup)(struct serial_private *, 
			 const struct pciserial_board *,
			 struct uart_port *, int);
	void	(*exit)(struct pci_dev *dev);
};

/*
 * This is the configuration table for all of the PCI serial boards
 * which we support.  It is directly indexed by the advpci_board_num_t enum
 * value, which is encoded in the pci_device_id PCI probe table's
 * driver_data member.
 *
 * The makeup of these names are:
 *  pbn_bn{_bt}_n_baud
 *
 *  bn   = PCI BAR number
 *  bt   = Index using PCI BARs
 *  n    = number of serial ports
 *  baud = baud rate
 */
enum advpci_board_num_t {
	adv_8port = 0,
	adv_4port,
	adv_2port,
};

static struct pciserial_board advpciserial_boards[] __devinitdata = {
	[adv_2port] = {
		.flags		    = FL_BASE0,
		.num_ports	    = 2,
		.base_baud	    = 7812500,
		.uart_offset	= 0x400,
		.reg_shift	    = 0,
		.first_offset	= 0,
	},

	[adv_4port] = {
		.flags		    = FL_BASE0,
		.num_ports	    = 4,
		.base_baud	    = 7812500,
		.uart_offset	= 0x400,
		.reg_shift	    = 0,
		.first_offset	= 0,
	},

	[adv_8port] = {
		.flags		    = FL_BASE0,
		.num_ports	    = 8,
		.base_baud	    = 7812500,
		.uart_offset	= 0x400,
		.reg_shift	    = 0,
		.first_offset	= 0,
	},
};

/*
 * Configuration:
 *   share_irqs - whether we pass SA_SHIRQ to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
#define SERIALEXAR_SHARE_IRQS 1 
unsigned int share_irqs = SERIALEXAR_SHARE_IRQS;

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA

#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
struct uart_adv_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */

	unsigned char		lsr_saved_flags;
	unsigned char		msr_saved_flags;
	
	unsigned short 		deviceid;
	unsigned short 		board_id;
	unsigned char		channelnum;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
					  unsigned int state, unsigned int old);

	int 			index;			/* serial index in card, start at 0 */
	int				serialMode;		/* RS232 or RS422 or RS485 */

	int				current_card_nr;
	int				fix_number_index;
};

struct irq_info {
	struct			hlist_node node;
	int			irq;
	spinlock_t		lock;	/* Protects list not the hash */
	struct list_head	*head;
};

#define NR_IRQ_HASH		32	/* Can be adjusted later */
static struct hlist_head irq_lists[NR_IRQ_HASH];
static DEFINE_MUTEX(hash_mutex);	/* Used to walk the hash */

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
#define PORT_MAX_XR 1
#define XRPCIe_TYPE 1 // the second entry that is [1] in the array

static const struct serial_uart_config uart_config[PORT_MAX_XR+1] = {
	{ "Unknown",	1,	    0 },
	{ "ADV17v35x",	256,	0 },
};

/*++++++++++++++++++++++++++++++++  function declare  ++++++++++++++++++++++++++++++++*/

static int serialadv_setRS485(struct uart_port *port, int turnOn, int turnaroundTime);
static void serial_set_dtrdsr ( struct uart_port *port, int turn_on);
static void serial_set_ctsrts(struct uart_port *port, int turn_on);
static unsigned int
serialadv_calc_divisor(struct uart_port *port, unsigned int baud,
						unsigned int *quot, unsigned int *quot_fraction, unsigned int *devide1);

/*--------------------------------  function declare  --------------------------------*/

static int
setup_port(struct serial_private *priv, struct uart_port *port,
	   int bar, int offset, int regshift)
{
	struct pci_dev *dev = priv->dev;
	unsigned long base, len;

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	base = pci_resource_start(dev, bar);

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		len =  pci_resource_len(dev, bar);

		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = ioremap(base, len);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		port->iotype = UPIO_MEM;
		port->iobase = 0;
		port->mapbase = base + offset;
		port->membase = priv->remapped_bar[bar] + offset;
		port->regshift = regshift;
	} else {
		// got to be memory mapped. some hardware reading error?
		return -EINVAL;
	}
	return 0;
}


static int
pci_default_setup(struct serial_private *priv,
		  const struct pciserial_board *board,
		  struct uart_port *port, int idx)
{
	unsigned int bar, offset = board->first_offset;

	bar = FL_GET_BASE(board->flags);	
	offset += idx * board->uart_offset;

	return setup_port(priv, port, bar, offset, board->reg_shift);
}

/*
 * Master list of serial port init/setup/exit quirks.
 * This does not describe the general nature of the port.
 * (ie, baud base, number and location of ports, etc)
 *
 * This list is ordered alphabetically by vendor then device.
 * Specific entries must come before more generic entries.
 */
static struct pci_serial_quirk pci_serial_quirks[] = {	
	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_1602,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},
	
	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_1604,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_1610,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_1612,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_1620,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_1622,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_0014,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_0015,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCIE_0016,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCM_3612I,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},
	{
		.vendor		= ADVANTECH_VID,
		.device		= PCI_1602,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,	
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCI_1604,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCI_1610,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCI_1612,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCI_1620,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	},

	{
		.vendor		= ADVANTECH_VID,
		.device		= PCI_1622,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_default_setup,
	},
};

static _INLINE_ int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	for (quirk = pci_serial_quirks; ; quirk++)
		if (quirk_id_matches(quirk->vendor, dev->vendor) &&
		    quirk_id_matches(quirk->device, dev->device))		    
		 	break;
	return quirk;
}

static _INLINE_ unsigned int serial_in(struct uart_adv_port *up, int offset)
{
	return readb(up->port.membase + offset);
}

static _INLINE_ void
serial_out(struct uart_adv_port *up, int offset, int value)
{
	writeb(value, up->port.membase + offset);
}

static int serial_get_mode(struct uart_adv_port *up)
{
    unsigned char mpiolvl_l = 0, mpiolvl_h = 0;
	unsigned char serialMode = SERIAL_MODE_UNKNOWN;
	unsigned char mpioData = 0;
	int	index=0;
   
	// all MPIO set as input
	serial_out(up, XR_17V35X_MPIOSEL_L, 0xFF);
	serial_out(up, XR_17V35X_MPIOSEL_H, 0xFF);
	
	mpiolvl_l = serial_in(up, XR_17V35X_MPIOLVL_L);
	mpiolvl_h = serial_in(up, XR_17V35X_MPIOLVL_H);
    
	mpioData = up->index<4 ? mpiolvl_l : mpiolvl_h;
	index = up->index<4 ? up->index : up->index-4;	

	if ( mpioData & (1<<(index*2)) )
	{
	    serialMode = SERIAL_MODE_RS232;
	}
	else if ( mpioData & (1<<(index*2+1)) )
	{
	    serialMode = SERIAL_MODE_RS485ORRS422S;
	}
	else
	{
	    serialMode = SERIAL_MODE_RS422M;
	}
	DEBUG_INTR(KERN_INFO "serialMode = %d\n", serialMode);
	up->serialMode = serialMode;
	
	return serialMode;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
static void serialadv_stop_tx(struct uart_port *port, unsigned int tty_stop)
#else
static void serialadv_stop_tx(struct uart_port *port)
#endif
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;

	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
static void serialadv_start_tx(struct uart_port *port, unsigned int tty_stop)
#else
static void serialadv_start_tx(struct uart_port *port)
#endif
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;

	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serialadv_stop_rx(struct uart_port *port)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void serialadv_enable_ms(struct uart_port *port)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void
receive_chars(struct uart_adv_port *up, unsigned int *status)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)  // prevent compile warning

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_struct *tty = up->port.state->port.tty;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	struct tty_struct *tty = up->port.info->port.tty;
#else 
	struct tty_struct *tty = up->port.info->tty;
#endif

#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 9, 0)	
	struct uart_port *port = &up->port;
#endif	
	unsigned char ch[256], lsr = *status;
	char flag;
	int i, datasize_in_fifo = 0, port_index;	
	while(datasize_in_fifo!=serial_in(up, XR_17V35X_RXFIFO_CNT))
	/*Read Receive Fifo count until we get read same value twice*/
		datasize_in_fifo=serial_in(up, XR_17V35X_RXFIFO_CNT);

	port_index = up->port.line;
	flag = TTY_NORMAL;
  
	if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE | UART_LSR_FE | UART_LSR_OE))) 
	{
		/*
		* Mask off conditions which should be ignored.
		*/
		lsr &= up->port.read_status_mask;

		if (lsr & UART_LSR_BI) 
		{
			DEBUG_INTR("handling break....");
			flag = TTY_BREAK;
		} 
		else if (lsr & UART_LSR_PE)
		{
			flag = TTY_PARITY;
			printk("handling port<%d> Parity error....\n",port_index);			
		}
		else if (lsr & UART_LSR_FE)
		{
		    DEBUG_INTR("handling Frame error....\n");	
			flag = TTY_FRAME;
		}
	}
	
	//memcpy_fromio(ch, up->port.membase + UART_17V35X_RX_OFFSET, datasize_in_fifo);
    for(i=0;i<datasize_in_fifo;i++)
    {
      ch[i]= serial_in(up, XR_17v35x_UART_RHR);
    }
	up->port.icount.rx+=datasize_in_fifo;
	
	for(i = 0; i < datasize_in_fifo; i++)
	{
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 21)
	    if (uart_handle_sysrq_char(&up->port, ch[i], NULL)) 
	#else
		if (uart_handle_sysrq_char(&up->port, ch[i])) 
	#endif
		{
			continue;
		}
		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch[i], flag);
	}
    spin_unlock(&up->port.lock);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 9, 0)
	tty_flip_buffer_push(&port->state->port);
#else
	tty_flip_buffer_push(tty);
#endif
	spin_lock(&up->port.lock);

	DEBUG_INTR(" LSR_DR...");
}

static void transmit_chars(struct uart_adv_port *up)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32) 
	struct circ_buf *xmit = &up->port.info->xmit;
#else
	struct circ_buf *xmit = &up->port.state->xmit;
#endif
	int count, bytes_in_fifo;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 16)
	int tmp;
#endif

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
		serialadv_stop_tx(&up->port, 0);
#else
		serialadv_stop_tx(&up->port);
#endif
		return;
	}
	if (uart_circ_empty(xmit)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
		serialadv_stop_tx(&up->port, 0);
#else
		serialadv_stop_tx(&up->port);
#endif
		return;
	}

	bytes_in_fifo = serial_in(up, XR_17V35X_TXFIFO_CNT);
	// read the fifo count untill we get the same value twice
	while (bytes_in_fifo != serial_in(up, XR_17V35X_TXFIFO_CNT))
		bytes_in_fifo = serial_in(up, XR_17V35X_TXFIFO_CNT);

	// how much buffer is availabe now to write?	
	count = up->port.fifosize - bytes_in_fifo;
	
	if (uart_circ_chars_pending(xmit) < count)
		count = uart_circ_chars_pending(xmit);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
	if(count > 0)
	{
		//serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		memcpy_toio(up->port.membase + UART_17V35X_TX_OFFSET, &(xmit->buf[xmit->tail]), 1);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}
#else
	do
	{	
		// if the count is more than (tail to end of the buffer), transmit only the rest here.
		// tail+tmp&(UART_XMIT_SIZE-1) will reset the tail to the starting of the circular buffer
		if( ((xmit->tail + count) & (UART_XMIT_SIZE-1)) < xmit->tail)
		{			
			tmp = UART_XMIT_SIZE - xmit->tail;
			memcpy_toio(up->port.membase + UART_17V35X_TX_OFFSET, &(xmit->buf[xmit->tail]), tmp);
			xmit->tail += tmp;
			xmit->tail &= (UART_XMIT_SIZE-1);
			up->port.icount.tx += tmp;
			count	-= tmp;
		}
		else
		{		
			memcpy_toio(up->port.membase + UART_17V35X_TX_OFFSET, &(xmit->buf[xmit->tail]), count);	
			xmit->tail += count;
			xmit->tail &= UART_XMIT_SIZE - 1;
			up->port.icount.tx += count;
			count = 0;
		}

	}while (count > 0);
#endif


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
		serialadv_stop_tx(&up->port, 0);
#else
		serialadv_stop_tx(&up->port);
#endif
}

static unsigned int check_modem_status(struct uart_adv_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27) 
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI) 
#else
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) 
#endif
	{
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32) 
		wake_up_interruptible(&up->port.info->delta_msr_wait);
#else
		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
#endif
	}

	return status;
}

/*
 * This handles the interrupt from one port.
 */
static void serialadv_handle_port(struct uart_adv_port *up)
{
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

	status = serial_in(up, UART_LSR);

	DEBUG_INTR("status = %x...", status);

	if (status & (UART_LSR_DR | UART_LSR_BI))
		receive_chars(up, &status);
	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * This is the serial driver's interrupt routine.
 *
 * Arjan thinks the old way was overly complex, so it got simplified.
 * Alan disagrees, saying that need the complexity to handle the weird
 * nature of ISA shared interrupts.  (This is a special exception.)
 *
 * In order to handle ISA shared interrupts properly, we need to check
 * that all ports have been serviced, and therefore the ISA interrupt
 * line has been de-asserted.
 *
 * This means we need to loop through all ports. checking that they
 * don't have an interrupt pending.
 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 18)
static irqreturn_t serialadv_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t serialadv_interrupt(int irq, void *dev_id)
#endif
{
	struct irq_info *i = dev_id;
	struct list_head *l, *end = NULL;
	int pass_counter = 0, handled = 0;

	DEBUG_INTR("serialadv_interrupt(%d)...", irq);

	spin_lock(&i->lock);

	l = i->head;
	do {
		struct uart_adv_port *up;
		unsigned int iir, lcr;

		up = list_entry(l, struct uart_adv_port, list);

		lcr = serial_in(up, UART_LCR);  // store value of LCR
		serial_out(up, UART_LCR, lcr & 0x7F); // ensure LCR bit-7=0 before reading UART_IIR
		iir = serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			serialadv_handle_port(up);

			handled = 1;

			end = NULL;
		} else if (end == NULL)
			end = l;

		serial_out(up, UART_LCR, lcr); // restore LCR
		l = l->next;

		if (l == i->head && pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR
				"serialadv: too much work for irq%d\n", irq);
			break;
		}
	} while (l != end);

	spin_unlock(&i->lock);

	DEBUG_INTR("end.\n");

	return IRQ_RETVAL(handled);
}

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_adv_port *up)
{
	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &up->list)
			i->head = i->head->next;
		list_del(&up->list);
	} else {
		BUG_ON(i->head != &up->list);
		i->head = NULL;
	}

	spin_unlock_irq(&i->lock);

	/* List empty so throw away the hash node */
	if (i->head == NULL) {
		hlist_del(&i->node);
		kfree(i);
	}
}

static int serial_link_irq_chain(struct uart_adv_port *up)
{
	struct hlist_head *h;
	struct hlist_node *n;
	struct irq_info *i;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 16)
	int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;
#else
	int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? SA_SHIRQ : 0;
#endif

	mutex_lock(&hash_mutex);

	h = &irq_lists[up->port.irq % NR_IRQ_HASH];

	hlist_for_each(n, h) {
		i = hlist_entry(n, struct irq_info, node);
		if (i->irq == up->port.irq)
			break;
	}

	if (n == NULL) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
		i = kmalloc(sizeof(struct irq_info), GFP_KERNEL);
		memset(i , 0, sizeof(struct irq_info));
#else
		i = kzalloc(sizeof(struct irq_info), GFP_KERNEL);
#endif
		if (i == NULL) {
			mutex_unlock(&hash_mutex);
			return -ENOMEM;
		}
		spin_lock_init(&i->lock);
		i->irq = up->port.irq;
		hlist_add_head(&i->node, h);
	}
	mutex_unlock(&hash_mutex);

	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add_tail(&up->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&up->list);
		i->head = &up->list;
		spin_unlock_irq(&i->lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		//irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED: 0;
		irq_flags |= up->port.irqflags;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
		irq_flags = up->port.flags & UPF_SHARE_IRQ ? SA_SHIRQ : 0;
#else
		irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;
#endif
		ret = request_irq(up->port.irq, serialadv_interrupt,
				  irq_flags, "ADVserial", i);
		if (ret < 0)
			serial_do_unlink(i, up);
	}

	return ret;
}

static void serial_unlink_irq_chain(struct uart_adv_port *up)
{
	struct irq_info *i = NULL;
	struct hlist_node *n;
	struct hlist_head *h;

	mutex_lock(&hash_mutex);

	h = &irq_lists[up->port.irq % NR_IRQ_HASH];

	hlist_for_each(n, h) {
		i = hlist_entry(n, struct irq_info, node);
		if (i->irq == up->port.irq)
			break;
	}

	BUG_ON(n == NULL);
	BUG_ON(i->head == NULL);

	if (list_empty(i->head))
		free_irq(up->port.irq, i);

	serial_do_unlink(i, up);
	mutex_unlock(&hash_mutex);
}

static inline int poll_timeout(int timeout)
{
	return timeout > 6 ? (timeout / 2 - 2) : 1;
}

/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
static void serialadv_timeout(unsigned long data)
{
	struct uart_adv_port *up = (struct uart_adv_port *)data;
	unsigned int iir;

	iir = serial_in(up, UART_IIR);
	if (!(iir & UART_IIR_NO_INT))
		serialadv_handle_port(up);
	mod_timer(&up->timer, jiffies + poll_timeout(up->port.timeout));
}

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

static unsigned int serialadv_tx_empty(struct uart_port *port)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned long flags;
	unsigned int lsr;

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return (lsr & BOTH_EMPTY) == BOTH_EMPTY ? TIOCSER_TEMT : 0;
}

static unsigned int serialadv_get_mctrl(struct uart_port *port)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned int status;
	unsigned int ret;

	status = check_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serialadv_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void serialadv_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serialadv_startup(struct uart_port *port)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned long flags;
	int retval;

	up->capabilities = uart_config[up->port.type].flags;

	serial_out(up, XR_17V35X_EXTENDED_EFR, UART_EFR_ECB);
	serial_out(up, UART_IER, 0);

	/* Set the RX/TX trigger levels */
	/* These are some default values, the OEMs can change these values
		* according to their best case scenarios */
	
	serial_out(up, XR_17V35X_EXTENDED_RXTRG, 32);
	serial_out(up, XR_17V35X_EXTENDED_TXTRG, 64);


	/* Hysteresis level of 8 */
	serial_out(up, XR_17V35X_EXTENDED_FCTR, XR_17V35X_FCTR_TRGD | XR_17V35X_FCTR_RTS_8DELAY);

	serial_out(up, UART_LCR, 0);

	/* Wake up and initialize UART */
	serial_out(up, XR_17V35X_EXTENDED_EFR, UART_EFR_ECB | 0x10/*Enable Shaded bits access*/);
	serial_out(up,XR_17V35X_UART_MSR, 0);
	serial_out(up, UART_IER, 0);
	serial_out(up, UART_LCR, 0);

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in set_termios())
	 */
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);
	
	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	if(port->irq) {
		retval = serial_link_irq_chain(up);
		if(retval)
			return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
		
	/*
	* Most PC uarts need OUT2 raised to enable interrupts.
	*/
	if (is_real_interrupt(up->port.irq))
		up->port.mctrl |= TIOCM_OUT2;
	//to enable intenal loop, uncomment the line below
	//up->port.mctrl |= TIOCM_LOOP;

	serialadv_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	return 0;
}

static void serialadv_shutdown(struct uart_port *port)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned long flags;

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	
	up->port.mctrl &= ~TIOCM_OUT2;

	serialadv_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
				  UART_FCR_CLEAR_RCVR |
				  UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(up, UART_RX);

	if (port->irq)
		serial_unlink_irq_chain(up);
}

static unsigned int serialadv_get_divisor(struct uart_port *port, unsigned int baud, unsigned int *quot, unsigned int *quot_fraction, unsigned int *devide1)
{
	serialadv_calc_divisor(port, baud, quot, quot_fraction, devide1);

	return 0;
}

static void
serialadv_set_special_baudrate(struct uart_port *port, unsigned int special_baudrate)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	signed int baud, quot, quot_fraction;
	unsigned char lcr_bak;
	unsigned int reg_read;
	int port_index;
	unsigned int devide1;
	/* for baud rate settings */
	unsigned char   oldEFR = 0;
	unsigned char   oldMCR = 0;
	unsigned char   valof4xreg = 0, valof8xreg= 0;

	port_index = up->port.line;
	quot_fraction = 0;
	printk(KERN_INFO "Enter in serialadv_set non-standard baudrate:%d\n",special_baudrate);

	baud = special_baudrate/*uart_get_baud_rate(port, termios, old, 0, port->uartclk/4)*/;
    lcr_bak = serial_in(up, UART_LCR);
	    	
	if((port_index > 15)||(port_index < 0))
	{
	   return;
	}
	if(port_index >= 8) port_index = port_index - 8;

	serialadv_get_divisor(port, baud, &quot, &quot_fraction, &devide1);

	// set baud rate
	// set sample 16
	valof8xreg = serial_in(up, XR_17V35X_8XMODE);
	valof4xreg = serial_in(up, XR_17V35X_4XMODE);
	valof4xreg &= ~(1 << up->index);
	valof8xreg &= ~(1 << up->index);
	serial_out(up, XR_17V35X_8XMODE, valof8xreg);
	serial_out(up, XR_17V35X_4XMODE, valof4xreg);

	oldEFR = serial_in(up, XR_17V35X_EXTENDED_EFR);
	serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR|0x10);
	if(devide1)		//Prescaler Divide by 1
	{
		oldMCR = serial_in(up, UART_MCR);
		oldMCR &= ~UART_MCR_CLKSEL; // using Prescaler Divide by 1
		up->port.mctrl &= ~UART_MCR_CLKSEL;
		serial_out (up, UART_MCR, oldMCR);
	}
	else			//Prescaler Divide by 4
	{
		oldMCR = serial_in(up, UART_MCR);
		oldMCR |= UART_MCR_CLKSEL;  // using Prescaler Divide by 4
		up->port.mctrl |= UART_MCR_CLKSEL;
		serial_out (up, UART_MCR, oldMCR);
	}
	serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR);

    serial_out(up, UART_LCR, lcr_bak | UART_LCR_DLAB);/* set DLAB */
    serial_out(up, UART_DLL, quot & 0xff);		/* LS of divisor */
    serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */

	/* if RS485,we set DLD[7] */
	quot_fraction = up->serialMode == SERIAL_MODE_RS485ORRS422S ? ((quot_fraction&0x0f)|0x80) : (quot_fraction&0x0f);

	//Fractional baud rate support
	{
	    reg_read=(serial_in(up, XR_17V35X_UART_DLD)&0xF0);
	    DEBUG_INTR(KERN_INFO "serialadv_set_special_baudrate: quot =0x%x quot_fraction=0x%x DLD_reg=0x%x\n",quot,quot_fraction,reg_read);		
	    serial_out(up, XR_17V35X_UART_DLD, quot_fraction | reg_read);
	    reg_read=serial_in(up, XR_17V35X_UART_DLD);
	 }
	 serial_out(up, UART_LCR, lcr_bak);		/* reset DLAB */

}

static unsigned int serialadv_calc_divisor(struct uart_port *port, unsigned int baud, unsigned int *quot, unsigned int *quot_fraction, unsigned int *devide1)
{
	// calculate the most appropriate baud,
	// Prescaler divide by 1 or 4
	unsigned long     AppropriateInt_Decuple0 = 0, AppropriateInt_Decuple1 = 0;
	unsigned long     baudWithPrescaler0 = 0, baudWithPrescaler1 = 0;
	unsigned long     deviation = 0;
	unsigned long     deviationPersent0 = 0, deviationPersent1 = 0;
	unsigned long     tempclock = 125000000;
	unsigned long     samplingrate = 16;
	unsigned short    tmpBaudInt0 = 0, tmpBaudInt1 = 0;
	unsigned short    tmpBaudFraction0 = 0, tmpBaudFraction1 = 0;

	//calculate in prescaler Divide by 1
	AppropriateInt_Decuple0 = (unsigned long) (tempclock * 10 /(samplingrate * baud) );
	DEBUG_INTR(KERN_INFO"AppropriateInt_Decuple0 = %ld, tempclock = %ld, baud = %d, samplingrate = %ld\n",
			AppropriateInt_Decuple0, tempclock, baud, samplingrate);
	tmpBaudInt0 = (unsigned short)TRUNC(AppropriateInt_Decuple0);
	tmpBaudFraction0 = (unsigned short)ROUND ((AppropriateInt_Decuple0 -TRUNC(AppropriateInt_Decuple0) *10)*16);
	baudWithPrescaler0 = (tempclock * 10) /((tmpBaudInt0 * 10 + (tmpBaudFraction0 * 10/16))*samplingrate) ;
	deviation = baudWithPrescaler0 > baud ? (baudWithPrescaler0 - baud) : (baud -baudWithPrescaler0 ) ;
	deviationPersent0 = deviation * 100 / baud;
	DEBUG_INTR(KERN_INFO"deviationPersent0 = %ld, tmpBaudInt0 = 0x%x, tmpBaudFraction0 = 0x%x\n",
			deviationPersent0, tmpBaudInt0, tmpBaudFraction0);

	//calculate in prescaler Divide by 4
	tempclock = tempclock / 4;
	AppropriateInt_Decuple1 = (unsigned long) (tempclock * 10 /(samplingrate * baud) );
	tmpBaudInt1 = (unsigned short)TRUNC(AppropriateInt_Decuple1);
	tmpBaudFraction1 = (unsigned short) ROUND ((AppropriateInt_Decuple1 -TRUNC(AppropriateInt_Decuple1)*10 )*16);
	baudWithPrescaler1 = (tempclock * 10) / ((tmpBaudInt1 * 10 + (tmpBaudFraction1 * 10/16))*samplingrate);
	deviation = baudWithPrescaler1 > baud ? (baudWithPrescaler1 - baud) : (baud -baudWithPrescaler1 ) ;
	deviationPersent1 = deviation * 100 / baud;
	DEBUG_INTR(KERN_INFO"deviationPersent1 = %ld, tmpBaudInt1 = 0x%x, tmpBaudFraction1 = 0x%x\n",
			deviationPersent1, tmpBaudInt1, tmpBaudFraction1);

	/*
	 *  clk is 125000000, samplingrate is 16, quot is 16bits data,
	 *  calculate rule: quot = clk / (16*baud)
	 *  if baud<120, quot will be overrun, so we need devide clk by 4.
	 *  Though we set tmpBaudInt0, tmpBaudInt1 as unsinged short to prevent overrun, it's better to check it as 120.
	 */
	if(baud < 120)
	{
		*quot = tmpBaudInt1;
		*quot_fraction = tmpBaudFraction1;
		*devide1 = 0;
	}
	else
	{
		//find which one is  less deviation
		if ( deviationPersent0 > deviationPersent1)
		{
			*quot = tmpBaudInt1;
			*quot_fraction = tmpBaudFraction1;
			*devide1 = 0;
		}
		else
		{
			*quot = tmpBaudInt0;
			*quot_fraction = tmpBaudFraction0;
			*devide1 = 1;
		}
	}

	return 0;
}

static void serial_set_ctsrts(struct uart_port *port, int turn_on)
{
	unsigned char old_efr, old_mcr;
	struct uart_adv_port *up = (struct uart_adv_port *)port;

	old_efr = serial_in(up, XR_17V35X_EXTENDED_EFR);

	if(turn_on)
	{
		old_efr |= 0xc0;
		serial_out(up, XR_17V35X_EXTENDED_EFR, old_efr);
		
		old_mcr = serial_in(up, UART_MCR);
		old_mcr |= 0x02;
		serial_out(up, UART_MCR, (unsigned char) old_mcr);
	}
	else
	{
		old_efr &= 0x3f;
		serial_out(up, XR_17V35X_EXTENDED_EFR, old_efr);

		/* Force RTS# output to a HIGH (default) */
		old_mcr = serial_in(up, UART_MCR);
		old_mcr &= 0xfd;
		serial_out(up, UART_MCR, (unsigned char) old_mcr);
	}
}

static void serial_set_dtrdsr(struct uart_port *port, int turn_on)
{
	unsigned char old_efr, old_mcr;
	struct uart_adv_port *up = (struct uart_adv_port *)port;

	old_efr = serial_in(up, XR_17V35X_EXTENDED_EFR);
	
	if(turn_on)
	{
		old_efr |= 0xc0;
		serial_out(up, XR_17V35X_EXTENDED_EFR, old_efr);

		old_mcr = serial_in(up, UART_MCR);
		old_mcr |= 0x05;
		serial_out(up, UART_MCR, (unsigned char) old_mcr);
	}
	else
	{
		old_efr &= 0x3f;
		serial_out(up, XR_17V35X_EXTENDED_EFR, old_efr);

		/* Force DTR# output to a HIGH (default) */
		old_mcr = serial_in(up, UART_MCR);
		old_mcr &= 0xfe;
		serial_out(up, UART_MCR, (unsigned char) old_mcr);
	}
}

static void
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
serialadv_set_termios(struct uart_port *port, struct termios *termios,
		       struct termios *old)
#else
serialadv_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
#endif
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned char cval;
	unsigned long flags;
	unsigned int baud, quot, quot_fraction;
	unsigned int devide1 = 0;
	unsigned int reg_read;
	/* for baud rate settings */
	unsigned char   oldEFR = 0;
	unsigned char   oldMCR = 0;
	unsigned char   valof4xreg = 0, valof8xreg= 0;

	int port_index;
	port_index = up->port.line;
	quot_fraction = 0;

	switch (termios->c_cflag & CSIZE) 
	{
	case CS5:
		cval = 0x00;
		break;
	case CS6:
		cval = 0x01;
		break;
	case CS7:
		cval = 0x02;
		break;
	default:
	case CS8:
		cval = 0x03;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= 0x04;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 * Add limited max baud rate as 921600. 
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, 921600); 

	if((port_index > 15)||(port_index < 0))
	{
		return;
	}
	if(port_index >= 8) port_index = port_index - 8;

	serialadv_get_divisor(port, baud, &quot, &quot_fraction, &devide1);
	DEBUG_INTR(KERN_INFO"serialadv_get_divisor::baud =%d, quot=0x%x, quot_fraction=0x%x, devide1=%d\n", baud, quot, quot_fraction, devide1);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_PE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);

	serial_set_ctsrts(port, 0);
	serial_set_dtrdsr(port, 0);
	if(termios->c_cflag & CRTSCTS)
	{
		serial_set_ctsrts(port, 1);
	}
	else if(termios->c_cflag & CDTRDSR)
	{
		serial_set_dtrdsr(port, 1);
	}

	/*
	*	Auto XON/XOFF software flow control flags
	*/
	reg_read=serial_in(up, XR_17V35X_EXTENDED_EFR);
	if(((termios->c_iflag) & IXOFF)&&((termios->c_iflag) & IXON))
	{
		serial_out(up, XR_17V35X_UART_XON1,0x11); //Initializing XON1
		serial_out(up, XR_17V35X_UART_XOFF1,0x13); //Initializing XOFF1
		serial_out(up, XR_17V35X_EXTENDED_EFR, (reg_read & 0xF0) | 0x0A );
		printk(KERN_INFO "Software Flow Control Enabled\n");
	}
	else 
	{
		serial_out(up, XR_17V35X_EXTENDED_EFR, (reg_read) & 0xF0 );
		printk(KERN_INFO "No Software Flow Control\n");
	}	

	reg_read=serial_in(up, XR_17V35X_EXTENDED_EFR);
	if((termios->c_iflag) & IXANY)
	{
		serial_out(up, XR_17V35X_EXTENDED_EFR, ((termios->c_iflag) & IXOFF)&&((termios->c_iflag) & IXON)?((reg_read) | 0x2A):((reg_read) | 0x20));
		reg_read=serial_in(up, UART_MCR);
		serial_out(up, UART_MCR, ((reg_read) & 0xDF) | 0x20 );
		reg_read=serial_in(up,XR_17V35X_EXTENDED_EFR);
		serial_out(up, XR_17V35X_EXTENDED_EFR, (reg_read) & 0xEF );
		printk(KERN_INFO "AUTO XANY Enabled\n");
	}
	else 
	{
		serial_out(up, XR_17V35X_EXTENDED_EFR, (reg_read) | 0x10 );
		reg_read=serial_in(up, UART_MCR);
		serial_out(up, UART_MCR, (reg_read) & 0xDF );
		reg_read=serial_in(up,XR_17V35X_EXTENDED_EFR);
		serial_out(up, XR_17V35X_EXTENDED_EFR, (reg_read) & 0xEF );
		printk(KERN_INFO "AUTO XANY NOT Enabled\n");
	}
	
//---------------------------------------------------------------------------//

	if(!((baud == 38400) && ((port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)))
	{
		// set baud rate
		// set sample 16
		valof8xreg = serial_in(up, XR_17V35X_8XMODE);
		valof4xreg = serial_in(up, XR_17V35X_4XMODE);
		valof4xreg &= ~(1 << up->index);
		valof8xreg &= ~(1 << up->index);
		serial_out(up, XR_17V35X_8XMODE, valof8xreg);
		serial_out(up, XR_17V35X_4XMODE, valof4xreg);

		oldEFR = serial_in(up, XR_17V35X_EXTENDED_EFR);
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR|0x10);
		if(devide1)  	//Prescaler Divide by 1
		{
			oldMCR = serial_in(up, UART_MCR);
			oldMCR &= ~UART_MCR_CLKSEL; // using Prescaler Divide by 1
			up->port.mctrl &= ~UART_MCR_CLKSEL;
			serial_out (up, UART_MCR, oldMCR);
		}
		else		//Prescaler Divide by 4
		{
			oldMCR = serial_in(up, UART_MCR);
			oldMCR |= UART_MCR_CLKSEL;  // using Prescaler Divide by 4
			up->port.mctrl |= UART_MCR_CLKSEL;
			serial_out (up, UART_MCR, oldMCR);
		}
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR);


		serial_out(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */

		serial_out(up, UART_DLL, (unsigned char)(quot & 0xff));		/* LS of divisor */
		serial_out(up, UART_DLM, (unsigned char)(quot >> 8));		/* MS of divisor */

		/* if RS485,we set DLD[7] */
		quot_fraction = up->serialMode == SERIAL_MODE_RS485ORRS422S ? ((quot_fraction&0x0f)|0x80) : (quot_fraction&0x0f);

		//Fractional baud rate support
		reg_read=(serial_in(up, XR_17V35X_UART_DLD)&0xF0);
		DEBUG_INTR(KERN_INFO "serialxr_set_termios: quot =0x%x quot_fraction=0x%x DLD_reg=0x%x\n",quot,quot_fraction,reg_read);		
		serial_out(up, XR_17V35X_UART_DLD, quot_fraction | reg_read);

		serial_out(up, UART_LCR, cval);				/* reset DLAB */
		up->lcr = cval;						/* Save LCR */
	}
	
	// set RS485
	up->serialMode == SERIAL_MODE_RS485ORRS422S ? serialadv_setRS485(&up->port, 1, 0) : serialadv_setRS485(&up->port, 0, 0);

	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);/* set fcr */
	/*
		Configuring MPIO as inputs
	*/
	if((up->deviceid == 0x354)||(up->deviceid == 0x4354)||(up->deviceid == 0x8354))
	{
		serial_out(up, XR_17V35x_MPIOSEL_7_0,0x0FF); //0x0ff= ALL INPUTS	
	}
	else if((up->deviceid == 0x358)||(up->deviceid == 0x4358)||(up->deviceid == 0x8358))
	{
		serial_out(up, XR_17V35x_MPIOSEL_7_0,0x0FF); //0x0ff= ALL INPUTS
		serial_out(up, XR_17V35x_MPIOSEL_15_8,0x0FF); //0x0ff= ALL INPUTS
	}
	
	serialadv_set_mctrl(&up->port, up->port.mctrl);
#if ENABLE_INTERNAL_LOOPBACK
	reg_read=serial_in(up, UART_MCR);
	serial_out(up, UART_MCR, (reg_read) | 0x10);
	printk(KERN_INFO "Enabling Internal Loopback\n");
#endif
	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 *      EXAR ioctls
 */
//#define 	FIOQSIZE		0x5460 
#define		EXAR_READ_REG      	(FIOQSIZE + 1)
#define 	EXAR_WRITE_REG     	(FIOQSIZE + 2)

#define 	EXAR_SET_MULTIDROP_MODE_NORMAL   (FIOQSIZE + 3)
#define 	EXAR_SET_MULTIDROP_MODE_AUTO     (FIOQSIZE + 4)
#define 	EXAR_SET_REMOVE_MULTIDROP_MODE   (FIOQSIZE + 5)



struct advioctl_rw_reg {
	unsigned char reg;
	unsigned char regvalue;
};
/*
 * This function is used to handle Advantech Device specific ioctl calls
 * The user level application should have defined the above ioctl
 * commands with the above values to access these ioctls and the 
 * input parameters for these ioctls should be struct advioctl_rw_reg
 * The Ioctl functioning is pretty much self explanatory here in the code,
 * and the register values should be between 0 to XR_17V35X_EXTENDED_RXTRG
 */

static int
serialadv_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	int ret = -ENOIOCTLCMD;
	struct advioctl_rw_reg ioctlrwarg;
	int iArg = 0;
	switch (cmd)
	{
		case ADV_READ_REG:
			if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
				return -EFAULT;
			ioctlrwarg.regvalue = serial_in(up, ioctlrwarg.reg);
			if (copy_to_user((void *)arg, &ioctlrwarg, sizeof(ioctlrwarg)))
				return -EFAULT;
			DEBUG_INTR(KERN_INFO "serialadv_ioctl read reg[0x%02x]=0x%02x \n",ioctlrwarg.reg,ioctlrwarg.regvalue);
			ret = 0;
			break;
		
		case ADV_WRITE_REG:
			if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
				return -EFAULT;
			serial_out(up, ioctlrwarg.reg, ioctlrwarg.regvalue);
			DEBUG_INTR(KERN_INFO "serialadv_ioctl write reg[0x%02x]=0x%02x \n",ioctlrwarg.reg,ioctlrwarg.regvalue);
			ret = 0;
			break;

		// serial have RS232/RS422/RS485 mode, get it
		case ADV_GET_SERIAL_MODE:
			DEBUG_INTR(KERN_INFO"enter ADV_GET_SERIAL_MODE\n");		
			if ( copy_to_user((void *)arg, &up->serialMode, sizeof(int)) )
				return -EFAULT;
			ret = 0;
			break;

		case ADV_SET_TURN_AROUND_DELAY:
			DEBUG_INTR(KERN_INFO"enter ADV_SET_TURN_AROUND_DELAY\n");
			iArg = (int) arg;

			DEBUG_INTR(KERN_INFO"iArg = %d\n", iArg);
			if( (up->serialMode!=SERIAL_MODE_RS485ORRS422S) || iArg<0 || iArg>15)
				return -EFAULT;

			serialadv_setRS485(&up->port, 1, iArg);
			ret = 0;
			break;
	}
	
	return ret;
}
	      
static void
serialadv_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned char oldEFR = 0;
	oldEFR = serial_in(up, XR_17V35X_EXTENDED_EFR);
	if (state) {
		/* sleep */
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR | 0x10);
		serial_out(up, UART_IER, UART_IERX_SLEEP);
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR &(~0x10));
	} else {
		/* wake */
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR | 0x10);
		serial_out(up, UART_IER, 0);
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR &(~0x10));
	}

	if (up->pm)
		up->pm(port, state, oldstate);
}

static void serialadv_release_port(struct uart_port *port)
{	
}

static int serialadv_request_port(struct uart_port *port)
{
	return 0;
}

static void serialadv_config_port(struct uart_port *port, int flags)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;	

	if (flags & UART_CONFIG_TYPE)
	{	
		up->port.type = XRPCIe_TYPE;
		if((uart_config[up->port.type].dfl_xmit_fifo_size >> sizeof(up->port.fifosize)) != 0 )
			up->port.fifosize =  0xff;
		else
			up->port.fifosize = uart_config[up->port.type].dfl_xmit_fifo_size;
		up->capabilities = uart_config[up->port.type].flags;	
	}
}

static const char *
serialadv_type(struct uart_port *port)
{
	int type = port->type;
	
	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static int serialadv_setRS485(struct uart_port *port, int turnOn, int turnaroundTime)
{
	struct uart_adv_port *up = (struct uart_adv_port *)port;
	unsigned char oldFCTR 	= 0;
	unsigned char oldDLD 	= 0;
	unsigned char oldLCR	= 0;
	unsigned char oldEFR	= 0;
	unsigned char valueMSR 	= 0;
	DEBUG_INTR("enter serialadv_setRS485\n");
	oldFCTR = serial_in(up, XR_17V35X_EXTENDED_FCTR);
	if(turnOn)	// set RS485
	{
		// Auto RS485 Enable 
		oldFCTR |= 0x20;
		serial_out(up, XR_17V35X_EXTENDED_FCTR, oldFCTR);
		
		// set RS-485 Polarity to 1
		oldLCR = serial_in(up, UART_LCR);
		serial_out(up, UART_LCR, oldLCR|UART_LCR_DLAB);
		oldDLD = serial_in(up, XR_17V35X_UART_DLD);
		serial_out(up, XR_17V35X_UART_DLD, oldDLD|0x80);
		serial_out(up, UART_LCR, oldLCR);	

		/* 
		 * set RS-485 turn-around delay
		 */
		oldEFR = serial_in(up, XR_17V35X_EXTENDED_EFR);
		oldEFR |= 0x10;
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR);	
		
		// MSR write only
		valueMSR = turnaroundTime<<4;
		serial_out(up, UART_MSR, valueMSR);	

		oldEFR &= ~(0x10);
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR);	
	}
	else		// unset RS485
	{
		// Auto RS485 Disable
		oldFCTR &= 0xdf;
		serial_out(up, XR_17V35X_EXTENDED_FCTR, oldFCTR);

		// restore msr	
		oldEFR = serial_in(up, XR_17V35X_EXTENDED_EFR);
		oldEFR |= 0x10;
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR);	
		
		// MSR write only
		valueMSR = 0;
		serial_out(up, UART_MSR, valueMSR);

		oldEFR &= ~(0x10);
		serial_out(up, XR_17V35X_EXTENDED_EFR, oldEFR);	
	}
	DEBUG_INTR("leave serialadv_setRS485\n");
	return 0;
}

void serialadv_spi_delay(void)
{
	int idex = 10;

	while(idex-- > 0)
	{
		;
	}

	return;
}

void serialadv_spi_write_slave(struct uart_adv_port *up, unsigned char data)
{
	int				idex		= 0;
	int				send_bit	= 0;
	unsigned char	reg_value	= 0;

	for(idex = 7; idex >= 0; idex--)
	{
		send_bit = data & (1<<idex) ? 1 : 0;
		serial_out(up, REGB, 0);					/* CK low */
		serialadv_spi_delay();
		reg_value = send_bit ? EEDI_BIT : 0;
		serial_out(up, REGB, reg_value);			/* write EEDI bit */
		serialadv_spi_delay();
		serial_out(up, REGB, reg_value | EECK_BIT);	/* CK high */
		serialadv_spi_delay();
		serial_out(up, REGB, 0);					/* CK low */
		serialadv_spi_delay();
	}

	return;
}

int serialadv_spi_read_slave(struct uart_adv_port *up, unsigned char *p_data)
{
	int				idex		= 0;
	int				recv_bit	= 0;
	unsigned char	reg_value	= 0;

	if(!p_data)
	{
		DEBUG_INTR("p_data == NULL\n");
		return -1;
	}

	*p_data = 0;
	for(idex = 7; idex >= 0; idex--)
	{
		serial_out(up, REGB, 0);			/* CK low */
		serialadv_spi_delay();
		serial_out(up, REGB, EECK_BIT);		/* CK high */
		serialadv_spi_delay();
		serial_out(up, REGB, 0);			/* CK high */
		serialadv_spi_delay();

		reg_value = serial_in(up, REGB);	/* read EEDO bit */
		recv_bit = reg_value & EEDO_BIT ? 1 : 0;
		*p_data |= recv_bit << idex;
	}

	return 0;
}

void serialadv_spi_start(struct uart_adv_port *up)
{
	/*
	 * SPI start, set CS low
	 */
	serial_out(up, XR_17V35X_MPIOSEL_H, 0x7F);
	serial_out(up, XR_17V35X_MPIOLVL_H, 0x0);
	serialadv_spi_delay();

	return;
}

void serialadv_spi_stop(struct uart_adv_port *up)
{
	/*
	 * SPI start, set CS low
	 */
	serial_out(up, XR_17V35X_MPIOSEL_H, 0x7F);
	serial_out(up, XR_17V35X_MPIOLVL_H, 0x80);
	serialadv_spi_delay();

	return;
}

/*
 * Routine Description:
 *    This routine will get card's board ID by SPI bus.
 * Arguments:
 *    up          - Pointer to the device structure
 *    p_boardID   - save board ID
 *Return value:
 *    0 on success, something else appropriate on failure
 */
int serialadv_get_boardID(struct uart_adv_port *up, short unsigned int *p_boardID)
{
	unsigned char boardID = 0;

	serialadv_spi_start(up);
	serialadv_spi_write_slave(up, SPI_READ_CMD);
	serialadv_spi_write_slave(up, GPIOA);
	serialadv_spi_read_slave(up, &boardID);
	serialadv_spi_stop(up);
	*p_boardID = boardID;

	return 0;
}

static int serialadv_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	printk(KERN_INFO "serialadv_set_special_baudrate flags:%d custom_divisor=%d\n",ser->flags,ser->custom_divisor);
	if(ser->custom_divisor > 0)
		serialadv_set_special_baudrate(port,ser->baud_base/ser->custom_divisor);
	return ret;
}

static struct uart_ops serialadv_pops = {
	.tx_empty	    = serialadv_tx_empty,
	.set_mctrl	    = serialadv_set_mctrl,
	.get_mctrl	    = serialadv_get_mctrl,
	.stop_tx	    = serialadv_stop_tx,
	.start_tx	    = serialadv_start_tx,
	.stop_rx	    = serialadv_stop_rx,
	.enable_ms	    = serialadv_enable_ms,
	.break_ctl	    = serialadv_break_ctl,
	.startup	    = serialadv_startup,
	.shutdown	    = serialadv_shutdown,
	.set_termios	= serialadv_set_termios,
	.verify_port	= serialadv_verify_port, 
	.pm		        = serialadv_pm,
	.type		    = serialadv_type,
	.release_port	= serialadv_release_port,
	.request_port	= serialadv_request_port,
	.config_port	= serialadv_config_port,
	.ioctl		    = serialadv_ioctl,
};

static DEFINE_MUTEX(serial_mutex);

static struct uart_adv_port serialadv_ports[NR_PORTS];

#define SERIALADV_CONSOLE	NULL

static struct uart_driver adv_uart_driver[MAX_CARD_SUPPORT];
static char adv_dev_name[MAX_CARD_SUPPORT][MAX_STRING_LEN];
static char adv_driver_name[MAX_CARD_SUPPORT][MAX_STRING_LEN];
unsigned int old_method_cnt = 0;

static struct uart_adv_port *serialadv_find_match_or_unused(struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.
	 */
	for (i = 0; i < NR_PORTS; i++)
		if (uart_match_port(&serialadv_ports[i].port, port))
			return &serialadv_ports[i];

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < NR_PORTS; i++)
		if (serialadv_ports[i].port.type == PORT_UNKNOWN &&
		    serialadv_ports[i].port.iobase == 0)
		{
			port->line = i;
			return &serialadv_ports[i];
		}

	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < NR_PORTS; i++)
		if (serialadv_ports[i].port.type == PORT_UNKNOWN)
			return &serialadv_ports[i];

	return NULL;
}

void init_adv_uart_struct(void)
{
	int i;

	/*
	 * We do not init "dev_name", "nr" here, these will be
	 * initiated after getting boardID.
	 */
	for(i = 0; i < MAX_CARD_SUPPORT; i++)
	{
		adv_uart_driver[i].owner		= THIS_MODULE;
		adv_uart_driver[i].driver_name	= "ADVserial";
		adv_uart_driver[i].major		= XR_MAJOR;
		adv_uart_driver[i].minor		= XR_MINOR + i;
		adv_uart_driver[i].cons			= SERIALADV_CONSOLE;
	}

	return;
}

/*
 *	serialadv_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int serialadv_register_port(struct uart_port *port, unsigned short deviceid,
								int index, int nr_ports)
{
	struct uart_adv_port *uart;
	int i, boardID = 0, ret = -ENOSPC;
	char tmp_string[MAX_STRING_LEN], tmp_dev_name[MAX_STRING_LEN];

	if (port->uartclk == 0)
		return -EINVAL;

	mutex_lock(&serial_mutex);
	uart = serialadv_find_match_or_unused(port);
	if (uart) {
		uart->port.iobase   = port->iobase;
		uart->port.membase  = port->membase;
		uart->port.irq      = port->irq;
		uart->port.uartclk  = port->uartclk;
		uart->port.fifosize = port->fifosize;
		uart->port.regshift = port->regshift;
		uart->port.iotype   = port->iotype;
		uart->port.flags    = port->flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase  = port->mapbase;
		if (port->dev)
			uart->port.dev = port->dev;

		uart->deviceid = deviceid;
		uart->channelnum = index;
		uart->port.line = port->line;
		uart->index = index;
		spin_lock_init(&uart->port.lock);

		init_timer(&uart->timer);
		uart->timer.function = serialadv_timeout;

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		uart->mcr_mask = ~(0x0); //~ALPHA_KLUDGE_MCR;
		uart->mcr_force = 0; // ALPHA_KLUDGE_MCR;

		uart->port.ops = &serialadv_pops;		

		/*
		 * Get serial mode
		 */
		serial_get_mode(uart);

		/*
		 * Get board ID
		 */
		for(i = 0; i < 10; i++)
		{
			serialadv_get_boardID(uart, &uart->board_id);
			if(uart->board_id != 0 && uart->board_id == boardID)
				break;
			boardID = uart->board_id;
		}

		uart->current_card_nr = nr_ports;
		if(uart->board_id == 0x0)
		{
			/*
			 * Use old naming methods "ttyAP*"
			 * WARNING: DO NOT try to modify this value "state" anywhere,
			 * this is private value for kernel
			 */
			if(adv_uart_driver[0].state == NULL)
			{
				/*
				 * The first member adv_uart_driver[0] is reserved for "ttyAP*"
				 */
				memset(adv_dev_name[0], 0, sizeof(adv_dev_name[0]));
				strncpy(adv_dev_name[0], "ttyAP", sizeof(adv_dev_name[0]) - 1);
				adv_uart_driver[0].dev_name = adv_dev_name[0];
				adv_uart_driver[0].nr = NR_PORTS;

				uart_register_driver(&adv_uart_driver[0]);
			}

			/*
			 * Add this port to first member
			 */
			uart->port.line = old_method_cnt++;
			uart->fix_number_index = 0;
			ret = uart_add_one_port(&adv_uart_driver[0], &uart->port);
		}
		else
		{
			/*
			 * Enable fixed number.
			 * Fill temp buffer, "ttyB" + boardID + "P" + port_num
			 */
			memset(tmp_dev_name, 0, sizeof(tmp_dev_name));
			strncpy(tmp_dev_name, "ttyB", sizeof(tmp_dev_name) - 1);
			sprintf(tmp_string, "%02d", uart->board_id);
			strncat(tmp_dev_name, tmp_string,
					 sizeof(tmp_dev_name) - strlen(tmp_dev_name));
			strncat(tmp_dev_name, "P",
					 sizeof(tmp_dev_name) - strlen(tmp_dev_name));

			/*
			 * The first member is reserved for "ttyAP*", begin to loop from 1
			 */
			for(i = 1; i < MAX_CARD_SUPPORT; i++)
			{
				/*
				 * Try to find filled member out first
				 */
				if(adv_uart_driver[i].state != NULL
					&& strlen(adv_uart_driver[i].dev_name) > 0
					&& strlen(tmp_dev_name) > 0)
				{
					if(strcmp(adv_uart_driver[i].dev_name, tmp_dev_name) == 0)
					{
						uart->port.line = index;
						uart->fix_number_index = i;
						ret = uart_add_one_port(&adv_uart_driver[i], &uart->port);
						break;
					}
				}

				/*
				 * Try to find& init the first uninitial member at this step
				 */
				if(adv_uart_driver[i].state == NULL)
				{
					/*
					 * Re-write index for port number
					 */
					adv_uart_driver[i].nr = nr_ports;

					/*
					 * Set device_name, "ttyB" + boardID + "P" + port_num
					 */
					memset(adv_dev_name[i], 0, sizeof(adv_dev_name[i]));
					strncpy(adv_dev_name[i], tmp_dev_name, sizeof(tmp_dev_name));
					adv_uart_driver[i].dev_name = adv_dev_name[i];

					memset(adv_driver_name[i], 0, sizeof(adv_driver_name[i]));
					sprintf(adv_driver_name[i], "ADVserialBID");
					strncat(adv_driver_name[i], tmp_string,
							 sizeof(adv_driver_name[i]) - strlen(adv_driver_name[i]));
					adv_uart_driver[i].driver_name = adv_driver_name[i];

					uart_register_driver(&adv_uart_driver[i]);

					uart->port.line = index;
					uart->fix_number_index = i;
					ret = uart_add_one_port(&adv_uart_driver[i], &uart->port);
					break;
				}
			}
		}
#if 0
		if (ret == 0)
		{
			ret = uart->port.line;

			if (is_real_interrupt(uart->port.irq)) {
				serial_link_irq_chain(uart);
			}
		}
#endif
	}
	mutex_unlock(&serial_mutex);

	return ret;
}

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int __devinit
init_one_advpciserialcard(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct serial_private *priv;
	struct pciserial_board *board;
	struct pci_serial_quirk *quirk;
	struct uart_port serial_port;
	int rc, nr_ports, i;
			
	if (ent->driver_data >= ARRAY_SIZE(advpciserial_boards)) {
		printk(KERN_INFO "pci_init_one: invalid driver_data: %ld\n",
			ent->driver_data);
		return -EINVAL;
	}

	board = &advpciserial_boards[ent->driver_data];

	rc = pci_enable_device(dev);
	if (rc)
		return rc;
	
	nr_ports = board->num_ports;

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0)
			goto disable;
		if (rc)
			nr_ports = rc;
	}

	priv = kmalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * nr_ports,
		       GFP_KERNEL);
	if (!priv) {
		rc = -ENOMEM;
		goto deinit;
	}

	memset(priv, 0, sizeof(struct serial_private) +
			sizeof(unsigned int) * nr_ports);

	priv->dev = dev;
	priv->quirk = quirk;

	memset(&serial_port, 0, sizeof(struct uart_port));
	serial_port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;

	serial_port.uartclk = board->base_baud*16;
	serial_port.irq = dev->irq;
	serial_port.dev = &dev->dev;
	for (i = 0; i < nr_ports; i++) {
		if (quirk->setup(priv, board, &serial_port, i))
			break;

		rc = serialadv_register_port(&serial_port, dev->device, i, nr_ports);
		if (rc < 0) {
			printk(KERN_WARNING "Couldn't register serial port %s: %d\n", pci_name(dev), i);
			break;
		}
				
		printk(KERN_WARNING "init_one_advpciserialcard line:%d\n",serial_port.line);
		priv->uart_index[i] = serial_port.line;
		priv->line[i] = rc;
	}

	priv->nr = i;

	if (!IS_ERR(priv)) {
		pci_set_drvdata(dev, priv);
		return 0;
	}

 deinit:
	if (quirk->exit)
		quirk->exit(dev);
 disable:
	pci_disable_device(dev);
	return rc;
}

/*
 *	serialadv_unregister_port - remove a serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void serialadv_unregister_port(int index, int line)
{
	struct uart_adv_port *uart = &serialadv_ports[line];

	mutex_lock(&serial_mutex);
#if 0
	if (is_real_interrupt(uart->port.irq))
	  serial_unlink_irq_chain(uart);
#endif
	if(adv_uart_driver[index].state != NULL)
	{
		uart_remove_one_port(&adv_uart_driver[index], &uart->port);
	}
	uart->port.dev = NULL;	
	mutex_unlock(&serial_mutex);
}

void pciserial_remove_ports(struct serial_private *priv)
{
	int i, current_index;
	struct pci_serial_quirk *quirk;

	/*
	 * Unregister ports one by one
	 */
	current_index =serialadv_ports[priv->uart_index[0]].fix_number_index;

	if((adv_uart_driver[current_index].state != NULL) && (adv_uart_driver[current_index].nr > 0))
	{
		for(i = 0; i < serialadv_ports[priv->uart_index[0]].current_card_nr; i++)
		{
			serialadv_unregister_port(current_index, priv->uart_index[i]);
		}

		if(current_index != 0)
			adv_uart_driver[current_index].nr = 0;
	}

	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (priv->remapped_bar[i])
			iounmap(priv->remapped_bar[i]);
		priv->remapped_bar[i] = NULL;
	}

	/*
	 * Find the exit quirks.
	 */
	quirk = find_quirk(priv->dev);
	if (quirk->exit)
		quirk->exit(priv->dev);

	kfree(priv);
}

static void __devexit remove_one_advpciserialcard(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	pci_set_drvdata(dev, NULL);

	pciserial_remove_ports(priv);

	pci_disable_device(dev);
}


static struct pci_device_id advserial_pci_tbl[] = {
	{	ADVANTECH_VID, PCIE_1602,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_2port },
	{	ADVANTECH_VID, PCIE_1604,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_2port },
	{	ADVANTECH_VID, PCIE_1610,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_4port },
	{	ADVANTECH_VID, PCIE_1612,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_4port },
	{	ADVANTECH_VID, PCIE_1620,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_8port },
	{	ADVANTECH_VID, PCIE_1622,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_8port },
	{	ADVANTECH_VID, PCIE_0014,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_8port },
	{	ADVANTECH_VID, PCIE_0015,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_4port },
	{	ADVANTECH_VID, PCIE_0016,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_2port },	
	{	ADVANTECH_VID, PCM_3612I,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_4port },
	{	ADVANTECH_VID, PCI_1602,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_2port },
	{	ADVANTECH_VID, PCI_1604,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_2port },
	{	ADVANTECH_VID, PCI_1610,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_4port },
	{	ADVANTECH_VID, PCI_1612,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_4port },
	{	ADVANTECH_VID, PCI_1620,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_8port },
	{	ADVANTECH_VID, PCI_1622,
		PCI_ANY_ID, PCI_ANY_ID,
		0, 0, adv_8port },        		
	{ 0, }
};

static struct pci_driver advserial_pci_driver = {
	.name		= "ADVserial",
	.probe		= init_one_advpciserialcard,
	.remove		= __devexit_p(remove_one_advpciserialcard),
	.id_table	= advserial_pci_tbl,
};


static int __init serialadv_init(void)
{
	int i, ret;

	printk("\n");
	printk("==========================================================="
			"====\n");
	printk("Advantech PCIe (17V35x) Serial Device Drivers.\n");
	printk("Product V%s [%s]\nFile V%s\n",
		ADVANTECH_EXAR_VER, ADVANTECH_EXAR_DATE, ADVANTECH_EXAR_FILE_VER);
	printk("Supports: RS232/422/485 auto detection and setting\n");
	printk("Devices:  ICOM: PCIE-1602, PCIE-1604\n"
			"                PCIE-1610, PCIE-1612\n"
			"                PCIE-1620, PCIE-1622\n"
			"                PCM_3612I\n"
			"                PCI-1602, PCI-1604\n"
			"                PCI-1610, PCI-1612\n"
			"                PCI-1620, PCI-1622\n"
			);
	printk("Advantech Industrial Automation Group.\n");
	printk("==========================================================="
			"====\n");

	init_adv_uart_struct();

	ret = pci_register_driver(&advserial_pci_driver);

	if (ret < 0)
	{
		for(i = 0; i < MAX_CARD_SUPPORT; i++)
		{
			if(adv_uart_driver[i].state != NULL)
			{
				uart_unregister_driver(&adv_uart_driver[i]);
			}
		}
	}

	return ret;	
}

static void __exit serialadv_exit(void)
{
	int i;

	pci_unregister_driver(&advserial_pci_driver);
	
	for(i = 0; i < MAX_CARD_SUPPORT; i++)
	{
		if(adv_uart_driver[i].state != NULL)
		{
			uart_unregister_driver(&adv_uart_driver[i]);
		}
	}
}

module_init(serialadv_init);
module_exit(serialadv_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advantech PCIe specific serial driver for ADV17V35X");
MODULE_VERSION(ADVANTECH_EXAR_VER);

MODULE_DEVICE_TABLE(pci, advserial_pci_tbl);
