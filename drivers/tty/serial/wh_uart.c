// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 UCTECHIP
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>

#include "wh_uart.h"

#define DRV_NAME "wh_uart"
#define SERIAL_WH_MAJOR 201
#define SERIAL_WH_MINOR 211

#define SERIAL_WH_UART_MAXPORTS     2


//*****************************************************************
//----------------------------start-----------------------------------

/*
 * Local per-uart structure.
 */
struct wh_uart {
	struct uart_port port;
	struct timer_list tmr;
	unsigned int sigs;	/* Local copy of line sigs */
	unsigned char imr;	/* Local IMR mirror */
};


/***********************   basic operations    ************/

static u8 wh_uart_readb(struct uart_port *port, int reg)
{
	return readb(port->membase + (reg << port->regshift));
}

static void wh_uart_writeb(struct uart_port *port, u8 dat, int reg)
{
	writeb(dat, port->membase + (reg << port->regshift));
}

static void wh_uart_putc(struct uart_port *port, u8 c)
{
	wh_uart_writeb(port,(wh_uart_readb(port, UART_CON) |(UART_CON_TRS)), UART_CON);
	wh_uart_writeb(port, c, UART_DATA);
	while(!(wh_uart_readb(port, UART_IS) & UART_IS_TXEND))
		cpu_relax();
	wh_uart_writeb(port, (wh_uart_readb(port, UART_IS) & (~UART_IS_TXEND)), UART_IS);
}

/************************************************************/

static unsigned int wh_uart_tx_empty(struct uart_port *port)
{
	return (wh_uart_readb(port, UART_IS) & UART_IS_FIFO_NE) ?  0 : TIOCSER_TEMT;
}



static void wh_uart_update_ctrl_reg(struct wh_uart *pp)
{
	unsigned char imr = pp->imr;

	/*
	 * If the device doesn't have an irq, ensure that the irq bits are
	 * masked out to keep the irq line inactive.
	 */
	if (!pp->port.irq)
		imr &= ~UART_IE_TXEND | ~UART_IE_FIFO_NE;

	wh_uart_writeb(&pp->port, imr, UART_IE);
}


static unsigned int wh_uart_get_mctrl(struct uart_port *port)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);
	unsigned int sigs = 0;

	//sigs = (wh_uart_readb(port, ALTERA_UART_STATUS_REG) &
	 //    ALTERA_UART_STATUS_CTS_MSK) ? TIOCM_CTS : 0;
	sigs |= (pp->sigs & TIOCM_RTS);

	return sigs;
}

static void wh_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);

	wh_uart_update_ctrl_reg(pp);
}

static void wh_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	wh_uart_update_ctrl_reg(pp);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void wh_uart_tx_chars(struct wh_uart *pp)
{
	struct uart_port *port = &pp->port;
	struct circ_buf *xmit = &port->state->xmit;



	if (port->x_char) {
		/* Send special char - probably flow control */
		wh_uart_putc(port, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

    wh_uart_writeb(port,(wh_uart_readb(port, UART_CON) |(UART_CON_TRS)), UART_CON);
	
	while (!(wh_uart_readb(port, UART_IS) & UART_IS_FIFO_NE)) {
		if (xmit->head == xmit->tail)
			break;
		wh_uart_putc(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (xmit->head == xmit->tail) {
		wh_uart_update_ctrl_reg(pp);
	}
}


static void wh_uart_start_tx(struct uart_port *port)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);

	pp->imr |= UART_IE_TXEND;
	wh_uart_update_ctrl_reg(pp);
	wh_uart_tx_chars(pp);
}

static void wh_uart_stop_tx(struct uart_port *port)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);

	pp->imr &= ~UART_IE_TXEND;
	wh_uart_update_ctrl_reg(pp);
}

static void wh_uart_stop_rx(struct uart_port *port)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);

	pp->imr &= ~UART_IE_FIFO_NE;
	wh_uart_update_ctrl_reg(pp);
}

static void wh_uart_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    struct ktermios *old)
{

	unsigned long flags;
	unsigned int baud;
	unsigned int br;
	char br_l,br_h;

	baud = uart_get_baud_rate(port, termios, old, 0, 4000000);
 
	if (old)
		tty_termios_copy_hw(termios, old);
	tty_termios_encode_baud_rate(termios, baud, baud);

	spin_lock_irqsave(&port->lock, flags);
	uart_update_timeout(port, termios->c_cflag, baud);

	br = port->uartclk /baud; //TODO:Now used the port->uartclk as the uart device clock for Debug, need to change into uart baud rate on ASIC
	br_l = (char)(br % 256);
	br_h = (char)(br / 256);
	wh_uart_writeb(port, br_l, UART_BPRL);
	wh_uart_writeb(port, (wh_uart_readb(port, UART_BPRH) & 0xF0) |br_h, UART_BPRH);

	spin_unlock_irqrestore(&port->lock, flags);

}

static void wh_uart_rx_chars(struct wh_uart *pp)
{
	struct uart_port *port = &pp->port;
	unsigned char ch, flag;

	wh_uart_writeb(port, wh_uart_readb(port, UART_CON) &~UART_CON_TRS, UART_CON);
	if(wh_uart_readb(port, UART_IS) & UART_IS_FIFO_NE){
		ch = wh_uart_readb(port, UART_DATA);
		flag = TTY_NORMAL;
		port->icount.rx++;
		uart_insert_char(port, 0, 0, ch, flag);
		}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
}



static irqreturn_t wh_uart_interrupt(int irq, void *data)
{
	struct uart_port *port = data;
	struct wh_uart *pp = container_of(port, struct wh_uart, port);
	unsigned int isr;

	isr = wh_uart_readb(port, UART_IS) & pp->imr;

	spin_lock(&port->lock);
	if (isr & UART_IS_FIFO_NE)
		wh_uart_rx_chars(pp);
	if (isr & UART_IS_TXEND)
		wh_uart_tx_chars(pp);
	spin_unlock(&port->lock);

	return IRQ_RETVAL(isr);
}

static void wh_uart_timer(struct timer_list *t)
{
	struct wh_uart *pp = from_timer(pp, t, tmr);
	struct uart_port *port = &pp->port;

	wh_uart_interrupt(0, port);
	mod_timer(&pp->tmr, jiffies + uart_poll_timeout(port));
}

static void wh_uart_config_port(struct uart_port *port, int flags)
{

	port->type = PORT_WH;
}

static int wh_uart_startup(struct uart_port *port)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);
	unsigned long flags;

	if (!port->irq) {
		timer_setup(&pp->tmr, wh_uart_timer, 0);
		mod_timer(&pp->tmr, jiffies + uart_poll_timeout(port));
	} else {
		int ret;

		ret = request_irq(port->irq, wh_uart_interrupt, 0,
				DRV_NAME, port);
		pr_err(DRV_NAME ": able to attach WH UART %d "
			       "interrupt vector=%d\n", port->line, port->irq);
		if (ret) {
			pr_err(DRV_NAME ": unable to attach WH UART %d "
			       "interrupt vector=%d\n", port->line, port->irq);
			return ret;
		}
	}

	spin_lock_irqsave(&port->lock, flags);

	/* Enable RX & TX interrupts now */
	pp->imr |= UART_IE_FIFO_NE;
	pp->imr |= UART_IE_TXEND;
	wh_uart_update_ctrl_reg(pp);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void wh_uart_shutdown(struct uart_port *port)
{
	struct wh_uart *pp = container_of(port, struct wh_uart, port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Disable all interrupts now */
	pp->imr = 0;
	wh_uart_update_ctrl_reg(pp);

	spin_unlock_irqrestore(&port->lock, flags);

	if (port->irq)
		free_irq(port->irq, port);
	else
		del_timer_sync(&pp->tmr);
}

static const char *wh_uart_type(struct uart_port *port)
{
	return (port->type == PORT_WH) ? "WH UART" : NULL;
}

static int wh_uart_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void wh_uart_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

static int wh_uart_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_WH))
		return -EINVAL;
	return 0;
}


#ifdef CONFIG_CONSOLE_POLL
static u8 wh_uart_poll_get_char(struct uart_port *port)
{
	return wh_uart_getc(port);
}

static void wh_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	wh_uart_putc(port, c);
}
#endif


/*
 *	Define the basic serial functions we support.
 */
static const struct uart_ops wh_uart_ops = {
	.tx_empty	= wh_uart_tx_empty,
	.get_mctrl	= wh_uart_get_mctrl,
	.set_mctrl	= wh_uart_set_mctrl,
	.start_tx	= wh_uart_start_tx,
	.stop_tx	= wh_uart_stop_tx,
	.stop_rx	= wh_uart_stop_rx,
	.break_ctl	= wh_uart_break_ctl,
	.startup	= wh_uart_startup,
	.shutdown	= wh_uart_shutdown,
	.set_termios	= wh_uart_set_termios,
	.type		= wh_uart_type,
	.request_port	= wh_uart_request_port,
	.release_port	= wh_uart_release_port,
	.config_port	= wh_uart_config_port,
	.verify_port	= wh_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= wh_uart_poll_get_char,
	.poll_put_char	= wh_uart_poll_put_char,
#endif
};

static struct wh_uart wh_uart_ports[SERIAL_WH_UART_MAXPORTS];

#if defined(CONFIG_SERIAL_WH_CONSOLE)
static void wh_uart_console_putc(struct uart_port *port, int c)
{
	wh_uart_putc(port, c);
}

static void wh_uart_console_write(struct console *co, const char *s,
				      unsigned int count)
{
	struct uart_port *port = &(wh_uart_ports + co->index)->port;
    
	uart_console_write(port, s, count, wh_uart_console_putc);
}

static int wh_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'E';
	int flow = 'n';

	if (co->index < 0 || co->index >= SERIAL_WH_UART_MAXPORTS)
		return -EINVAL;
	port = &wh_uart_ports[co->index].port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver wh_uart_driver;

static struct console wh_uart_console = {
	.name	= "ttyWH",
	.write	= wh_uart_console_write,
	.device	= uart_console_device,
	.setup	= wh_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &wh_uart_driver,
};

static int __init wh_uart_console_init(void)
{
	register_console(&wh_uart_console);
	return 0;
}

console_initcall(wh_uart_console_init);

#define	WH_UART_CONSOLE	(&wh_uart_console)

static void wh_uart_earlycon_write(struct console *co, const char *s,
				       unsigned int count)
{
	struct earlycon_device *dev = co->data;

	uart_console_write(&dev->port, s, count, wh_uart_console_putc);
}

static int __init wh_uart_earlycon_setup(struct earlycon_device *dev,
					     const char *options)
{
	struct uart_port *port = &dev->port;

	if (!port->membase)
		return -ENODEV;

	/* Enable RX interrupts now */
	wh_uart_writeb(port, UART_IE_FIFO_NE, UART_IE);

	if (dev->baud) {
		//uctechip uart baudrate setting, fixed 115200
		unsigned int br;
		char br_l,br_h;
		br = port->uartclk /115200;
		br_l = (char)(br % 256);
		br_h = (char)(br / 256);
		wh_uart_writeb(port, br_l, UART_BPRL);
		wh_uart_writeb(port, (wh_uart_readb(port, UART_BPRH) & 0xF0) |br_h, UART_BPRH);
	}

	dev->con->write = wh_uart_earlycon_write;
	return 0;
}

OF_EARLYCON_DECLARE(uart, "wh,uart-1.0", wh_uart_earlycon_setup);

#else

#define	WH_UART_CONSOLE	NULL

#endif /* CONFIG_SERIAL_WH_UART_CONSOLE */


/*
 *	Define the wh_uart UART driver structure.
 */
static struct uart_driver wh_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRV_NAME,
	.dev_name	= "ttyWH",
	.major		= SERIAL_WH_MAJOR,
	.minor		= SERIAL_WH_MINOR,
	.nr		= SERIAL_WH_UART_MAXPORTS,
	.cons		= WH_UART_CONSOLE,
};

static int wh_uart_probe(struct platform_device *pdev)
{
	struct uart_port *port;
	struct resource *res_mem;
	int irq;
	int i = pdev->id;
	int ret;

	/* if id is -1 scan for a free id and use that one */
	if (i == -1) {
		for (i = 0; i < SERIAL_WH_UART_MAXPORTS; i++)
			if (wh_uart_ports[i].port.mapbase == 0)
				break;
	}

	if (i < 0 || i >= SERIAL_WH_UART_MAXPORTS)
		return -EINVAL;

	port = &wh_uart_ports[i].port;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res_mem)
		port->mapbase = res_mem->start;
	else
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	
	if (irq < 0)
		return -EPROBE_DEFER;
	else
		port->irq = irq;

	
	/* Check platform data first so we can override device node data */
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
					   &port->uartclk);
	if (ret)
		return ret;

	port->membase = ioremap(port->mapbase, 0x40);
	if (!port->membase)
		return -ENOMEM;

	port->regshift = 0;

	port->line = i;
	port->type = PORT_ALTERA_UART;
	port->iotype = SERIAL_IO_MEM;
	port->ops = &wh_uart_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = &pdev->dev;

	platform_set_drvdata(pdev, port);
   	uart_add_one_port(&wh_uart_driver, port);

	return 0;
}

static int wh_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	if (port) {
		uart_remove_one_port(&wh_uart_driver, port);
		port->mapbase = 0;
		iounmap(port->membase);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wh_uart_match[] = {
	{ .compatible = "wh,uart0", },
	{},
};
MODULE_DEVICE_TABLE(of, wh_uart_match);
#endif /* CONFIG_OF */

static struct platform_driver wh_uart_platform_driver = {
	.probe	= wh_uart_probe,
	.remove	= wh_uart_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(wh_uart_match),
	},
};

static int __init wh_uart_init(void)
{
	int rc;

    printk(KERN_INFO "UCTECHIP WH UART device driver\n");
	rc = uart_register_driver(&wh_uart_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&wh_uart_platform_driver);
	if (rc)
		uart_unregister_driver(&wh_uart_driver);
	return rc;
}

static void __exit wh_uart_exit(void)
{
	platform_driver_unregister(&wh_uart_platform_driver);
	uart_unregister_driver(&wh_uart_driver);
}

module_init(wh_uart_init);
module_exit(wh_uart_exit);

MODULE_DESCRIPTION("UCTECHIP WH UART driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_ALIAS_CHARDEV_MAJOR(SERIAL_WH_MAJOR);
