/*
 *  linux/drivers/mmc/wh_mmc.c - WH MMC driver
 */

#include <linux/module.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mmc/slot-gpio.h>

#include <linux/platform_data/mmc-wh.h>
#include "wh_mmc.h"
#include <linux/delay.h>

//#define ENTER_PRINT

#define DRIVER_NAME "wh_mmc"

#define WH_SD_ARGU                     0x00   // Command Argument Register (32,RW)
#define WH_SD_CMD                      0x04   // Command Setting Register (16,RW)
#define WH_SD_STATUS                   0x08   // Card Status Register (16,R)
#define WH_SD_RSP                      0x0c   // Command Response (32,R)
#define WH_SD_CS                       0x1c   // Controller Setting (Not in use) (16,R)
#define WH_SD_BSR                      0x20   // Block Size Register (16,R)
#define WH_SD_PCR                      0x24   // Power Control Register (8,R)
#define WH_SD_SWR                      0x28   // Software reset Register (8,RW)
#define WH_SD_TOR                      0x2c   // Timeout Register (16,RW)
#define WH_SD_NIS                      0x30   // Normal Interrupt Status Register (16,RW)
#define WH_SD_EIS                      0x34   // Error Interrupt Status Register (16,RW)
#define WH_SD_NIE                      0x38   // Normal Interrupt Enable (16,RW)
#define WH_SD_EIE                      0x3c   // Error Interrupt Enable Register (16,RW)
#define WH_SD_CAP                      0x48   // Capability Register (16,R)
#define WH_SD_CDR                      0x4c   // Clock Divider Register (8,RW)
#define WH_SD_BDBS					   0x50   // BD Status Register (16,RW)
#define WH_SD_DIS					   0x54   // Data Interrupt Status Register (16,RW)
#define WH_SD_DIE                      0x58   // Data Interrupt Enable Register (16,RW)
#define WH_SD_BDRX_L                   0x60   // BD RX (Low 32 bit) (32,W)
#define WH_SD_BDRX_H                   0x64   // BD RX (High 32 bit) (32,W)
#define WH_SD_BDTX_L                   0x80   // BD TX (Low 32 bit) (32,W)
#define WH_SD_BDTX_H                   0x84   // BD TX (High 32 bit) (32,W)
#define WH_SD_CUSTOM				   0x100  // CUSTOM  (16,RW)
#define WH_SD_RSP_2					   0x104
#define WH_SD_RSP_3					   0x108
#define WH_SD_RSP_4					   0x10c

//WordSelect(136bit Response)
#define WH_WORD_0 0x00
#define WH_WORD_1 0x40
#define WH_WORD_2 0x80
#define WH_WORD_3 0xC0

//Commands
#define WH_CMD2 0x200
#define WH_CMD3 0x300
#define WH_CMD7 0x700
#define WH_CMD8  0x800
#define WH_CMD9  0x900
#define WH_CMD13  0xD00
#define WH_CMD16  0x1000
#define WH_CMD17  0x1100

#define WH_ACMD41 0x2900
#define WH_ACMD6 0x600
#define WH_CMD55 0x3700

//CMD ARG
//CMD8
#define WH_VHS  0x100 //2.7-3.6V
#define WH_CHECK_PATTERN 0xAA
//ACMD41
#define WH_BUSY 0x80000000
#define WH_HCS 0x40000000
#define WH_VOLTAGE_MASK 0xFFFFFF

//CMD7
#define WH_READY_FOR_DATA   0x100
#define WH_CARD_STATUS_STB  0x600

//Command setting
#define WH_CICE 0x10
#define WH_CRCE 0x08
#define WH_RSP_48 0x2
#define WH_RSP_146 0x1

//Status Mask
//Card status
#define WH_CARD_BUSY 0x1

//Normal interupt status
#define WH_CMD_COMPLETE 0x1
#define WH_EI 0x8000

//Error interupt status
#define WH_CMD_TIMEOUT 0x1
#define WH_CCRC 0x1
#define WH_CIE  0x8

// Data Interrupt Status Register
#define WH_DATA_TRE		(1 << 4)
#define WH_DATA_CMDE	(1 << 3)
#define WH_DATA_FIFOE	(1 << 2)
#define WH_DATA_MRC		(1 << 1)
#define WH_DATA_TRS		(1 << 0)

#define WH_CID_MID_MASK 0x7F8000
#define WH_CID_OID_MASK 0x7FFF
#define WH_CID_B1 0x7F800000
#define WH_CID_B2 0x7F8000
#define WH_CID_B3 0x7F80
#define WH_CID_B4 0x7F

#define WH_RCA_RCA_MASK 0xFFFF0000


// Dcache regs and pointer
#define FLUSH_32 0x34005240
static void *dcache_reg;

enum dbg_channels {
	dbg_err   = (1 << 0),
	dbg_debug = (1 << 1),
	dbg_info  = (1 << 2),
	dbg_irq   = (1 << 3),
	dbg_sg    = (1 << 4),
	dbg_dma   = (1 << 5),
	dbg_pio   = (1 << 6),
	dbg_fail  = (1 << 7),
	dbg_conf  = (1 << 8),
};

static const int dbgmap_err   = dbg_fail;
static const int dbgmap_info  = dbg_info | dbg_conf;
static const int dbgmap_debug = dbg_err | dbg_debug;

#define dbg(host, channels, args...)		  \
	do {					  \
	if (dbgmap_err & channels) 		  \
		dev_err(&host->pdev->dev, args);  \
	else if (dbgmap_info & channels)	  \
		dev_info(&host->pdev->dev, args); \
	else if (dbgmap_debug & channels)	  \
		dev_dbg(&host->pdev->dev, args);  \
	} while (0)

static void finalize_request(struct wh_host *host);
static void wh_send_request(struct mmc_host *mmc);
static void wh_reset(struct wh_host *host);

#ifdef CONFIG_MMC_DEBUG

static void dbg_dumpregs(struct wh_host *host, char *prefix)
{
	u32 ARGU, CMD, STATUS, RSP, CS, BSR, PCR, SWR, TOR, NIS, EIS;
	u32 NIE, CAP, CDR, BDBS, DIS, DIE, CUSTOM ,RSP2 ,RSP3 ,RSP4;

	ARGU 	= readl(host->base + WH_SD_ARGU);
	CMD 	= readl(host->base + WH_SD_CMD);
	STATUS 	= readl(host->base + WH_SD_STATUS);
	RSP 	= readl(host->base + WH_SD_RSP);
	CS 		= readl(host->base + WH_SD_CS);
	BSR 	= readl(host->base + WH_SD_BSR);
	PCR 	= readl(host->base + WH_SD_PCR);
	SWR 	= readl(host->base + WH_SD_SWR);
	TOR 	= readl(host->base + WH_SD_TOR);
	NIS 	= readl(host->base + WH_SD_NIS);
	EIS 	= readl(host->base + WH_SD_EIS);
	NIE 	= readl(host->base + WH_SD_NIE);
	CAP 	= readl(host->base + WH_SD_CAP);
	CDR 	= readl(host->base + WH_SD_CDR);
	BDBS 	= readl(host->base + WH_SD_BDBS);
	DIS   	= readl(host->base + WH_SD_DIS);
	CUSTOM  = readl(host->base + WH_SD_CUSTOM);
	RSP2 	= readl(host->base + WH_SD_RSP_2);
	RSP3   	= readl(host->base + WH_SD_RSP_3);
	RSP4   	= readl(host->base + WH_SD_RSP_4);

	dbg(host, dbg_debug, "%s  ARGU:[%08x]  CMD:[%08x]  STATUS:[%08x]\n",
				prefix, ARGU, CMD, STATUS);

	dbg(host, dbg_debug, "%s RSP:[%08x] CS:[%08x] BSR:[%08x]\n",
				prefix, RSP, CS, BSR);

	dbg(host, dbg_debug, "%s PCR:[%08x] SWR:[%08x]"
			       " TOR:[%08x] NIS:[%08x]\n",
				prefix, PCR, SWR, TOR, NIS);

	dbg(host, dbg_debug, "%s   EIS:[%08x]   NIE:[%08x]"
			       "   CAP:[%08x]   CDR:[%08x]\n",
				prefix, EIS, NIE, CAP, CDR);

	dbg(host, dbg_debug, "%s  BDBS:[%08x]  DIS:[%08x]  DIE:[%08x]  CUSTOM:[%08x]\n",
					prefix, BDBS, DIS, DIE, CUSTOM);

	dbg(host, dbg_debug, "%s  RSP2:[%08x]  RSP3:[%08x]  RSP4:[%08x]\n",
						prefix, RSP2, RSP3, RSP4);
}

static void prepare_dbgmsg(struct wh_host *host, struct mmc_command *cmd,
			   int stop)
{
	snprintf(host->dbgmsg_cmd, 300,
		 "#%u%s op:%i arg:0x%08x flags:0x08%x retries:%u",
		 host->ccnt, (stop ? " (STOP)" : ""),
		 cmd->opcode, cmd->arg, cmd->flags, cmd->retries);

	if (cmd->data) {
		snprintf(host->dbgmsg_dat, 300,
			 "#%u bsize:%u blocks:%u bytes:%u",
			 host->dcnt, cmd->data->blksz,
			 cmd->data->blocks,
			 cmd->data->blocks * cmd->data->blksz);
	} else {
		host->dbgmsg_dat[0] = '\0';
	}
}

static void dbg_dumpcmd(struct wh_host *host, struct mmc_command *cmd,
			int fail)
{
	unsigned int dbglvl = fail ? dbg_fail : dbg_debug;

	if (!cmd)
		return;

	if (cmd->error == 0) {
		dbg(host, dbglvl, "CMD[OK] %s R0:0x%08x\n",
			host->dbgmsg_cmd, cmd->resp[0]);
	} else {
		dbg(host, dbglvl, "CMD[ERR %i] %s Status:%s\n",
			cmd->error, host->dbgmsg_cmd, host->status);
	}

	if (!cmd->data)
		return;

	if (cmd->data->error == 0) {
		dbg(host, dbglvl, "DAT[OK] %s\n", host->dbgmsg_dat);
	} else {
		dbg(host, dbglvl, "DAT[ERR %i] %s\n",
			cmd->data->error, host->dbgmsg_dat);
	}
}
#else
static void dbg_dumpcmd(struct wh_host *host,
			struct mmc_command *cmd, int fail) { }

static void prepare_dbgmsg(struct wh_host *host, struct mmc_command *cmd,
			   int stop) { }

static void dbg_dumpregs(struct wh_host *host, char *prefix) { }

#endif /* CONFIG_MMC_DEBUG */

/**
 * wh_host_usedma - return whether the host is using dma or pio
 * @host: The host state
 *
 * Return true if the host is using DMA to transfer data, else false
 * to use PIO mode. Will return static data depending on the driver
 * configuration.
 */
static inline bool wh_host_usedma(struct wh_host *host)
{
#ifdef CONFIG_MMC_WH_PIO
	return false;
#else /* CONFIG_MMC_WH_DMA */
	return true;
#endif
}

static inline u32 enable_imask(struct wh_host *host, u32 imask)
{
	u32 newmask;

	newmask = 0;

	return newmask;
}

static inline u32 disable_imask(struct wh_host *host, u32 imask)
{
	u32 newmask;

	newmask = 0;

	return newmask;
}

static inline void clear_imask(struct wh_host *host)
{

	writel(0, host->base + WH_SD_DIS);
	writel(0, host->base + WH_SD_EIS);
	writel(0, host->base + WH_SD_NIS);

}

/**
 * wh_check_sdio_irq - test whether the SDIO IRQ is being signalled
 * @host: The host to check.
 *
 * Test to see if the SDIO interrupt is being signalled in case the
 * controller has failed to re-detect a card interrupt. Read GPE8 and
 * see if it is low and if so, signal a SDIO interrupt.
 *
 * This is currently called if a request is finished (we assume that the
 * bus is now idle) and when the SDIO IRQ is enabled in case the IRQ is
 * already being indicated.
*/
static void wh_check_sdio_irq(struct wh_host *host)
{
	if (host->sdio_irqen) {
		{
			pr_debug("%s: signalling irq\n", __func__);
			mmc_signal_sdio_irq(host->mmc);
		}
	}

}

static inline int get_data_buffer(struct wh_host *host,
				  u32 *bytes, u32 **pointer)
{
	struct scatterlist *sg;

	//printk("Enter wh_mmc.c :get_data_buffer");

	if (host->pio_active == XFER_NONE)
		return -EINVAL;

	if ((!host->mrq) || (!host->mrq->data))
		return -EINVAL;

	if (host->pio_sgptr >= host->mrq->data->sg_len) {
		dbg(host, dbg_debug, "no more buffers (%i/%i)\n",
		      host->pio_sgptr, host->mrq->data->sg_len);
		return -EBUSY;
	}
	sg = &host->mrq->data->sg[host->pio_sgptr];

	*bytes = sg->length;
	*pointer = sg_virt(sg);

	host->pio_sgptr++;

	dbg(host, dbg_sg, "new buffer (%i/%i)\n",
	    host->pio_sgptr, host->mrq->data->sg_len);


	return 0;
}

static inline u32 fifo_count(struct wh_host *host)
{
	u32 fifostat =  32;//readl(host->base + WH_SD_BSR );

	return fifostat;
}

static inline u32 fifo_free(struct wh_host *host)
{
	u32 fifostat = 32;//readl(host->base + WH_SD_BDBS);

	fifostat &= 0xff;
	return fifostat;
}

/**
 * wh_enable_irq - enable IRQ, after having disabled it.
 * @host: The device state.
 * @more: True if more IRQs are expected from transfer.
 *
 * Enable the main IRQ if needed after it has been disabled.
 *
 * The IRQ can be one of the following states:
 *	- disabled during IDLE
 *	- disabled whilst processing data
 *	- enabled during transfer
 *	- enabled whilst awaiting SDIO interrupt detection
 */
static void wh_enable_irq(struct wh_host *host, bool more)
{
	unsigned long flags;
	bool enable = false;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_enable_irq");
#endif

	local_irq_save(flags);

	host->irq_enabled = more;
	host->irq_disabled = false;

	enable = more | host->sdio_irqen;

	if (host->irq_state != enable) {
		host->irq_state = enable;

		if (enable)
			enable_irq(host->irq);
		else
			disable_irq(host->irq);
	}

	local_irq_restore(flags);
}

/**
 *
 */
static void wh_disable_irq(struct wh_host *host, bool transfer)
{
	unsigned long flags;

#ifdef ENTER_PRINT
	printk("Enter wh_disable_irq");
#endif

	local_irq_save(flags);

	/* pr_debug("%s: transfer %d\n", __func__, transfer); */

	host->irq_disabled = transfer;

	if (transfer && host->irq_state) {
		host->irq_state = false;
		disable_irq(host->irq);
	}

	local_irq_restore(flags);
}

static void do_pio_read(struct wh_host *host)
{
	void __iomem *from_ptr;

	/* write real prescaler to host, it might be set slow to fix */

	from_ptr = host->data_buffer_viraddr;

	get_data_buffer(host, &host->pio_bytes, &host->pio_ptr);
	memcpy(host->pio_ptr,from_ptr,host->pio_bytes);
	host->pio_count += host->pio_bytes;
	host->pio_ptr += (host->pio_bytes/4);
	host->pio_bytes = 0;
	host->pio_active = XFER_NONE;
	host->complete_what = COMPLETION_FINALIZE;
	return;

}

static void do_pio_write(struct wh_host *host)
{
	void __iomem *to_ptr;

	#ifdef ENTER_PRINT
		printk("Enter do_pio_write");
	#endif
//	printk("host->mrq->cmd->arg %d",host->mrq->cmd->arg);

	to_ptr = host->data_buffer_viraddr;

	get_data_buffer(host, &host->pio_bytes, &host->pio_ptr);
	memcpy(to_ptr, host->pio_ptr, host->pio_bytes);
	host->pio_count += host->pio_bytes;
	host->pio_ptr += (host->pio_bytes/4);
	host->pio_bytes = 0;
	host->pio_active = XFER_NONE;
	host->complete_what = COMPLETION_FINALIZE;
	return ;
}

static void pio_tasklet(unsigned long data)
{
	struct wh_host *host = (struct wh_host *) data;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC pio_tasklet ,host->pio_active :%d",host->pio_active );
#endif


	wh_disable_irq(host, true);

	if (host->pio_active == XFER_WRITE)
		do_pio_write(host);

	if (host->pio_active == XFER_READ)
		do_pio_read(host);

	if (host->complete_what == COMPLETION_FINALIZE) {
		clear_imask(host);
		if (host->pio_active != XFER_NONE) {
			dbg(host, dbg_err, "unfinished %s "
			    "- pio_count:[%u] pio_bytes:[%u]\n",
			    (host->pio_active == XFER_READ) ? "read" : "write",
			    host->pio_count, host->pio_bytes);

			if (host->mrq->data)
				host->mrq->data->error = -EINVAL;
		}

		wh_enable_irq(host, false);
		finalize_request(host);
	} else
		wh_enable_irq(host, true);
}

/*
 * ISR for SDI Interface IRQ
 * Communication between driver and ISR works as follows:
 *   host->mrq 			points to current request
 *   host->complete_what	Indicates when the request is considered done
 *     COMPLETION_CMDSENT	  when the command was sent
 *     COMPLETION_RSPFIN          when a response was received
 *     COMPLETION_XFERFINISH	  when the data transfer is finished
 *     COMPLETION_XFERFINISH_RSPFIN both of the above.
 *   host->complete_request	is the completion-object the driver waits for
 *
 * 1) Driver sets up host->mrq and host->complete_what
 * 2) Driver prepares the transfer
 * 3) Driver enables interrupts
 * 4) Driver starts transfer
 * 5) Driver waits for host->complete_rquest
 * 6) ISR checks for request status (errors and success)
 * 6) ISR sets host->mrq->cmd->error and host->mrq->data->error
 * 7) ISR completes host->complete_request
 * 8) ISR disables interrupts
 * 9) Driver wakes up and takes care of the request
 *
 * Note: "->error"-fields are expected to be set to 0 before the request
 *       was issued by mmc.c - therefore they are only set, when an error
 *       contition comes up
 */

static irqreturn_t wh_irq(int irq, void *dev_id)
{
	struct wh_host *host = dev_id;
	struct mmc_command *cmd;
	u32  mci_dsta, mci_eis, mci_nis, mci_rsp;
	u32  mci_dclear;
	unsigned long iflags;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_irq ,irq=%d ,complete_what=%d",irq ,host->complete_what);
#endif

	mci_dsta = readl(host->base + WH_SD_DIS);
	mci_eis	 = readl(host->base + WH_SD_EIS);
	mci_nis	 = readl(host->base + WH_SD_NIS);
	mci_rsp  = readl(host->base + WH_SD_RSP);
#ifdef ENTER_PRINT
	printk("wh_irq :mci_dsta=%x ,mci_eis=%x ,mci_nis=%x ,rsp=%x", mci_dsta, mci_eis, mci_nis ,mci_rsp);
#endif

	spin_lock_irqsave(&host->complete_lock, iflags);

	mci_dclear = 0;

	if ((host->complete_what == COMPLETION_NONE) ||
	    (host->complete_what == COMPLETION_FINALIZE)) {
		host->status = "nothing to complete";
		clear_imask(host);
		goto irq_out;
	}

	if (!host->mrq) {
		host->status = "no active mrq";
		clear_imask(host);
		goto irq_out;
	}

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;

	if (!cmd) {
		host->status = "no active cmd";
		clear_imask(host);
		goto irq_out;
	}

	if (mci_eis & WH_CMD_TIMEOUT) {
		dbg(host, dbg_err, "CMDSTAT: error CMDTIMEOUT\n");
		cmd->error = -ETIMEDOUT;
		host->status = "error: command timeout";
		goto fail_transfer;
	}

	if (mci_nis & WH_CMD_COMPLETE) {
		if (host->complete_what == COMPLETION_CMDSENT) {
			host->status = "ok: command sent";
			goto close_transfer;
		}
	}

	if (mci_nis & WH_CMD_COMPLETE) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			host->status = "ok: command response received";
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;
	}

	/* errors handled after this point are only relevant
	   when a data transfer is in progress */

	if (!cmd->data)
		goto clear_status_bits;

	/* Check for FIFO failure */
	if (mci_dsta & WH_DATA_FIFOE) {
		dbg(host, dbg_err, "FIFO failure\n");
		host->mrq->data->error = -EILSEQ;
		host->status = "error: fifo failure";
		goto fail_transfer;
	}

	/* Check for CRC failure */
	if (mci_dsta & WH_DATA_TRE) {
		dbg(host, dbg_err, "bad data crc \n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc ";
		goto fail_transfer;
	}

	/* Check for Command failure */
	if (mci_dsta & WH_DATA_CMDE) {
		dbg(host, dbg_err, "Command error \n");
		cmd->data->error = -EILSEQ;
		host->status = "error: Command error ";
		goto fail_transfer;
	}

	/* Check for Max Retry Attempts reach failure */
	if (mci_dsta & WH_DATA_MRC) {
		dbg(host, dbg_err, "Max Retry Attempts reach failure \n");
		cmd->data->error = -EILSEQ;
		host->status = "error: Max Retry Attempts reach failure ";
		goto fail_transfer;
	}

	//ACMD51
	if (mci_nis & WH_CMD_COMPLETE & (host->mrq->cmd->opcode == 51) ) {
		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->status = "ok: data transfer completed";
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;

	}

	if (mci_dsta & WH_DATA_TRS) {
		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->status = "ok: data transfer completed";
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;

	}
	else {
		host->status = "error: dsta ,nis ,eis not set ";
		goto fail_transfer;
	}

clear_status_bits:
	clear_imask(host);
	goto irq_out;

fail_transfer:
	host->pio_active = XFER_NONE;
#ifdef	ENTER_PRINT
	printk("WH-MMC wh_irq:fail_transfer");
#endif

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;
#ifdef	ENTER_PRINT
	printk("WH-MMC wh_irq:close_transfer");
#endif
	clear_imask(host);
	tasklet_schedule(&host->pio_tasklet);

	goto irq_out;

irq_out:
#ifdef	ENTER_PRINT
	printk("WH-MMC wh_irq:irq_out,host->status=%s",host->status);
	//printk("WH-MMC wh_irq:host->status=%s",host->status);
#endif
	spin_unlock_irqrestore(&host->complete_lock, iflags);
	return IRQ_HANDLED;

}

static void wh_dma_done_callback(void *arg)
{
	struct wh_host *host = arg;
	unsigned long iflags;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_dma_done_callback");
#endif

	BUG_ON(!host->mrq);
	BUG_ON(!host->mrq->data);

	spin_lock_irqsave(&host->complete_lock, iflags);

	dbg(host, dbg_dma, "DMA FINISHED\n");

	host->dma_complete = 1;
	host->complete_what = COMPLETION_FINALIZE;

	tasklet_schedule(&host->pio_tasklet);
	spin_unlock_irqrestore(&host->complete_lock, iflags);

}

static void finalize_request(struct wh_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;
	int debug_as_failure = 0;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC finalize_request");
	printk("host->mrq->cmd->opcode :%d",host->mrq->cmd->opcode );
#endif


	if (host->complete_what != COMPLETION_FINALIZE)
		return;

	if (!mrq)
		return;
	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	if (cmd->data && (cmd->error == 0) &&
	    (cmd->data->error == 0)) {
		if (wh_host_usedma(host) && (!host->dma_complete)) {
			dbg(host, dbg_dma, "DMA Missing (%d)!\n",
			    host->dma_complete);
			return;
		}
	}

	/* Read response from controller. */
		cmd->resp[0] = readl(host->base + WH_SD_RSP);
		if(cmd->flags & MMC_RSP_136)
		{
			cmd->resp[1] = readl(host->base + WH_SD_RSP_2);
			cmd->resp[2] = readl(host->base + WH_SD_RSP_3);
			cmd->resp[3] = readl(host->base + WH_SD_RSP_4);
		}
		else
		{
			cmd->resp[1] = 0;
			cmd->resp[2] = 0;
			cmd->resp[3] = 0;
		}

	if (cmd->error)
		debug_as_failure = 1;

	if (cmd->data && cmd->data->error)
		debug_as_failure = 1;

	dbg_dumpcmd(host, cmd, debug_as_failure);

	/* Cleanup controller */
	writel(0, host->base + WH_SD_CUSTOM);
	clear_imask(host);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && (!host->cmd_is_stop)) {
		host->cmd_is_stop = 1;
		wh_send_request(host->mmc);
		return;
	}

	/* If we have no data transfer we are finished here */
	if (!mrq->data)
		goto request_done;

	/* Calculate the amout of bytes transfer if there was no error */
	if (mrq->data->error == 0) {
		mrq->data->bytes_xfered =
			(mrq->data->blocks * mrq->data->blksz);
	} else {
		mrq->data->bytes_xfered = 0;
	}

	/* If we had an error while transferring data we flush the
	 * DMA channel and the fifo to clear out any garbage. */
	if (mrq->data->error != 0) {
		if (wh_host_usedma(host))
			dmaengine_terminate_all(host->dma);

	}

request_done:
#ifdef ENTER_PRINT
	printk("WH-MMC finalize_request request_done");
#endif
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;

	wh_check_sdio_irq(host);
	mmc_request_done(host->mmc, mrq);
}

static void wh_send_command(struct wh_host *host,
					struct mmc_command *cmd)
{
	u32 ccon;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_send_command,host->base:%x",host->base);
	printk("WH-MMC wh_send_command cmd->arg: %d ", cmd->arg);
	printk("WH-MMC wh_send_command cmd->opcode(command index): %d ", cmd->opcode); //command index
	if(cmd->data)
		{
			printk("cmd->data->blk_addr: %d ", cmd->data->blk_addr);
			printk("cmd->data->blocks: %d ", cmd->data->blocks);
		}
#endif

//	enable_imask(host, imsk);

	if (cmd->data)
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

#ifdef ENTER_PRINT
	printk("WH-MMC wh_send_command ,host->complete_what :%d",host->complete_what);
#endif

	ccon  = cmd->opcode << 8;

	if (cmd->flags & MMC_RSP_PRESENT)
	{
		ccon |= WH_CICE ;
		ccon |= WH_CRCE ;
		ccon |= WH_RSP_48;
	}

	if (cmd->flags & MMC_RSP_136)
	{
		ccon &= (~WH_RSP_48);
		ccon |= WH_RSP_146;

	}
#ifdef ENTER_PRINT
	printk("WH-MMC wh_send_command ccon is %x",ccon);
#endif

	if (cmd->opcode == 51 )
		writel(0x3 ,host->base + WH_SD_CUSTOM);

	writel(ccon		, host->base + WH_SD_CMD);
	writel(cmd->arg	, host->base + WH_SD_ARGU);

}

static void wh_send_command_with_data(struct wh_host *host,
					struct mmc_command *cmd)
{
	int i;
#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_send_command_with_data");
	printk("WH-MMC wh_send_command_with_data cmd->arg: %d ", cmd->arg);
	printk("WH-MMC wh_send_command_with_data cmd->opcode(command index): %d ", cmd->opcode); //command index
	printk("WH-MMC wh_send_command_with_data phyaddr: %x,viraddr %x", host->data_buffer_phyaddr ,(u32)host->data_buffer_viraddr);
	if(cmd->data)
		{
			printk("cmd->data->blk_addr: %d ", cmd->data->blk_addr);
			printk("cmd->data->blocks: %d ", cmd->data->blocks);
		}
#endif

	if (cmd->data)
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	switch( cmd->opcode ){
	case 6://cmd6
		/* Enable Interrupt */
		wh_enable_irq(host, true);

		/* Enter Interrupt */
		wh_irq(11, host);

		break;

	case 13://acmd13
		/* Enable Interrupt */
		wh_enable_irq(host, true);

		/* Enter Interrupt */
		wh_irq(11, host);

		break;

	case 17:
		//asm("fence.i");
		for(i = 0; i < 8; i++)
			writel(((host->data_buffer_phyaddr + i * 0x40) >> 4), dcache_reg);
		writel(host->data_buffer_phyaddr ,host->base + WH_SD_BDRX_L);
		writel(host->data_buffer_phyaddr ,host->base + WH_SD_BDRX_L);
		writel(cmd->arg ,host->base + WH_SD_BDRX_L);
		writel(cmd->arg ,host->base + WH_SD_BDRX_L);
		wh_enable_irq(host, true);
		break;

	case 24:
		//asm("fence.i");
		for(i = 0; i < 8; i++)
                        writel(((host->data_buffer_phyaddr + i * 0x40) >> 4), dcache_reg);
		writel(host->data_buffer_phyaddr ,host->base + WH_SD_BDTX_L);
		writel(host->data_buffer_phyaddr ,host->base + WH_SD_BDTX_L);
		writel(cmd->arg ,host->base + WH_SD_BDTX_L);
		writel(cmd->arg ,host->base + WH_SD_BDTX_L);
		wh_enable_irq(host, true);
		break;

	case 51:
		writel(0x3 ,host->base + WH_SD_CUSTOM);

		break;

	default:

		break;

	}



}




static int wh_setup_data(struct wh_host *host, struct mmc_data *data)
{
	u32 stoptries = 3;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_setup_data");
#endif

	/* write DCON register */

	if (!data) {
		return 0;
	}

	if ((data->blksz & 3) != 0) {
		/* We cannot deal with unaligned blocks with more than
		 * one block being transferred. */

		if (data->blocks > 1) {
			pr_warn("%s: can't do non-word sized block transfers (blksz %d)\n",
				__func__, data->blksz);
			return -EINVAL;
		}
	}

	while (readl(host->base + WH_SD_STATUS) &
	       WH_CARD_BUSY) {

		dbg(host, dbg_err,
		    "mci_setup_data() transfer stillin progress.\n");

		wh_reset(host);

		if ((stoptries--) == 0) {
			dbg_dumpregs(host, "DRF");
			return -EINVAL;
		}
	}

	return 0;

}

#define BOTH_DIR (MMC_DATA_WRITE | MMC_DATA_READ)

static int wh_prepare_pio(struct wh_host *host, struct mmc_data *data)
{
	int rw = (data->flags & MMC_DATA_WRITE) ? 1 : 0;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_prepare_pio");
#endif

	BUG_ON((data->flags & BOTH_DIR) == BOTH_DIR);

	host->pio_sgptr = 0;
	host->pio_bytes = 0;
	host->pio_count = 0;
	host->pio_active = rw ? XFER_WRITE : XFER_READ;

	if (rw) {
		do_pio_write(host);
	}

	return 0;
}

static int wh_prepare_dma(struct wh_host *host, struct mmc_data *data)
{
	int rw = data->flags & MMC_DATA_WRITE;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config conf = {
		.src_addr = host->mem->start,	//reversed
		.dst_addr = host->mem->start,	//reversed
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
	};

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_prepare_dma");
#endif

	BUG_ON((data->flags & BOTH_DIR) == BOTH_DIR);

	if (!rw)
		conf.direction = DMA_DEV_TO_MEM;
	else
		conf.direction = DMA_MEM_TO_DEV;

	dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		   mmc_get_dma_dir(data));
	dmaengine_slave_config(host->dma, &conf);   //crash
	desc = dmaengine_prep_slave_sg(host->dma, data->sg, data->sg_len,
		conf.direction,
		DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc)
		goto unmap_exit;
	desc->callback = wh_dma_done_callback;
	desc->callback_param = host;
	dmaengine_submit(desc);
	dma_async_issue_pending(host->dma);

	return 0;

unmap_exit:
	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		     mmc_get_dma_dir(data));
	return -ENOMEM;
}

static void wh_send_request(struct mmc_host *mmc)
{
	struct wh_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_send_request");
	printk("WH-MMC cmd->arg: %d ", cmd->arg);
	printk("WH-MMC cmd->opcode(command index): %d ", cmd->opcode); //command index
#endif

	host->ccnt++;
	prepare_dbgmsg(host, cmd, host->cmd_is_stop);

	clear_imask(host);

	if (cmd->data) {
		int res = wh_setup_data(host, cmd->data);

		host->dcnt++;

		if (res) {
			dbg(host, dbg_err, "setup data error %d\n", res);
			cmd->error = res;
			cmd->data->error = res;

			mmc_request_done(mmc, mrq);
			return;
		}

		if (wh_host_usedma(host))
			res = wh_prepare_dma(host, cmd->data);
		else
			res = wh_prepare_pio(host, cmd->data);

		if (res) {
			dbg(host, dbg_err, "data prepare error %d\n", res);
			cmd->error = res;
			cmd->data->error = res;

			mmc_request_done(mmc, mrq);
			return;
		}


	}

	/* Send command */
	if (!cmd->data | (cmd->opcode ==51) ) {
		/*Enable nie,ese, Disable die*/
		writel(0x8001, host->base + WH_SD_NIE);
		writel(0x000B, host->base + WH_SD_EIE);
		writel(0x0000, host->base + WH_SD_DIE);

		wh_send_command(host, cmd);
		/* Enable Interrupt */
		wh_enable_irq(host, true);
	}
	else {
		/*Enable nie,ese, Disable die*/
		writel(0x0000, host->base + WH_SD_NIE);
		writel(0x0000, host->base + WH_SD_EIE);
		writel(0x001f, host->base + WH_SD_DIE);

		wh_send_command_with_data(host, cmd);
	}
	/* Delay to wait for command sent done*/
	//msleep_interruptible(1);

	/* Enable Interrupt */
//	wh_enable_irq(host, true);

	/* Enter Interrupt */
	//wh_irq(11, host);
}

static void wh_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct wh_host *host = mmc_priv(mmc);

	host->status = "mmc request";
	host->cmd_is_stop = 0;
	host->mrq = mrq;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_request");
//	printk("WH-MMC host->mrq->cmd->arg: %d ", host->mrq->cmd->arg);
//	printk("WH-MMC host->mrq->cmd->opcode(command index): %d ", host->mrq->cmd->opcode); //command index
	if(host->mrq->data)
	{
		printk("host->mrq->data->blk_addr: %d ", host->mrq->data->blk_addr);
		printk("host->mrq->data->blocks: %d ", host->mrq->data->blocks);
		printk("host->mrq->data->bytes_xfered: %d ", host->mrq->data->bytes_xfered);
		printk("host->mrq->data->blksz: %d ", host->mrq->data->blksz);
		//printk("host->mrq->data->bytes_xfered: %d ", host->mrq->data->);
	}
#endif

	wh_send_request(mmc);
}

static void wh_set_clk(struct wh_host *host, struct mmc_ios *ios)
{
	u32 mci_psc;
#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_set_clk");
#endif
	/* Set clock */
	for (mci_psc = 0; mci_psc < 255; mci_psc++) {
		host->real_rate = host->clk_rate / (host->clk_div*(mci_psc+1));

		if (host->real_rate <= ios->clock)
			break;
	}

	if (mci_psc > 255)
		mci_psc = 255;

	host->prescaler = mci_psc;

	/* If requested clock is 0, real_rate will be 0, too */
	if (ios->clock == 0)
		host->real_rate = 0;
}

static void wh_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct wh_host *host = mmc_priv(mmc);

#ifdef ENTER_PRINT
//	printk("Enter WH-MMC wh_set_ios");
//	printk("WH-MMC mmc_ios->clock:%d",ios->clock);
	//printk("WH-MMC mmc_ios->vdd:%d",ios->vdd);
	printk("WH-MMC mmc_ios->power_mode:%d",ios->power_mode);
	//printk("WH-MMC mmc_ios->bus_width:%d",ios->bus_width);
	if (host->pdata->set_power)
		printk("WH-MMC mmc_ios set_power is not empty");
#endif

	/* Set the Timeout Register */
	writel(0x8FFF, host->base + WH_SD_TOR);

	/* Set the Clock Divider */
	writel(0x1	, host->base + WH_SD_SWR);
	writel(0x0	, host->base + WH_SD_CDR);
	writel(0x0	, host->base + WH_SD_SWR);

	/* Set the Interrupt Enable */
	writel(0x8001, host->base + WH_SD_NIE);
	writel(0x000B, host->base + WH_SD_EIE);
	writel(0x001f, host->base + WH_SD_DIE);

	/* Set the power state */

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:

		/* Set the Timeout Register */
		writel(0x8FFF, host->base + WH_SD_TOR);

		/* Set the Clock Divider */
		writel(0x8FFF, host->base + WH_SD_TOR);
		writel(0x1	, host->base + WH_SD_SWR);
		writel(0x2	, host->base + WH_SD_CDR);
		writel(0x0	, host->base + WH_SD_SWR);

		/* Set the Interrupt Enable */
		writel(0x8001, host->base + WH_SD_NIE);
		writel(0x000B, host->base + WH_SD_EIE);
		writel(0x001f, host->base + WH_SD_DIE);

		break;

	case MMC_POWER_OFF:
	default:
		writel(0x1	, host->base + WH_SD_SWR);
		writel(0x0	, host->base + WH_SD_SWR);

		break;
	}

	wh_set_clk(host, ios);

	if ((ios->power_mode == MMC_POWER_ON) ||
	    (ios->power_mode == MMC_POWER_UP)) {
		dbg(host, dbg_conf, "running at %lukHz (requested: %ukHz).\n",
			host->real_rate/1000, ios->clock/1000);
	} else {
		dbg(host, dbg_conf, "powered down.\n");
	}

	host->bus_width = ios->bus_width;
}

static void wh_reset(struct wh_host *host)
{

	writel(0x1	, host->base + WH_SD_SWR);
	writel(0x2	, host->base + WH_SD_CDR);
	writel(0x0	, host->base + WH_SD_SWR);

}

static void wh_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct wh_host *host = mmc_priv(mmc);
	unsigned long flags;

#ifdef ENTER_PRINT
	printk("Enter WH-MMC wh_enable_sdio_irq");
#endif

	local_irq_save(flags);

	host->sdio_irqen = enable;

	if (enable == host->sdio_irqen)
		goto same_state;

	if (enable) {
		enable_imask(host, 1);

		if (!host->irq_state && !host->irq_disabled) {
			host->irq_state = true;
			enable_irq(host->irq);
		}
	} else {
		disable_imask(host, 1);

		if (!host->irq_enabled && host->irq_state) {
			disable_irq_nosync(host->irq);
			host->irq_state = false;
		}
	}

 same_state:
	local_irq_restore(flags);

	wh_check_sdio_irq(host);
}

static const struct mmc_host_ops wh_ops = {
	.request	= wh_request,
	.set_ios	= wh_set_ios,
	.get_ro		= mmc_gpio_get_ro,
	.get_cd		= mmc_gpio_get_cd,
	.enable_sdio_irq = wh_enable_sdio_irq,
};

static struct wh_mci_pdata wh_def_pdata = {
	/* This is currently here to avoid a number of if (host->pdata)
	 * checks. Any zero fields to ensure reasonable defaults are picked. */
	 .no_wprotect = 1,
	 .no_detect = 1,
};


static inline int wh_cpufreq_register(struct wh_host *host)
{
	return 0;
}

static inline void wh_cpufreq_deregister(struct wh_host *host)
{
}


#ifdef CONFIG_DEBUG_FS

static int wh_state_show(struct seq_file *seq, void *v)
{
	struct wh_host *host = seq->private;

	seq_printf(seq, "Register base = 0x%08x\n", (u32)host->base);
	seq_printf(seq, "Clock rate = %ld\n", host->clk_rate);
	seq_printf(seq, "Prescale = %d\n", host->prescaler);
	seq_printf(seq, "IRQ = %d\n", host->irq);
	seq_printf(seq, "IRQ enabled = %d\n", host->irq_enabled);
	seq_printf(seq, "IRQ disabled = %d\n", host->irq_disabled);
	seq_printf(seq, "IRQ state = %d\n", host->irq_state);
	seq_printf(seq, "CD IRQ = %d\n", host->irq_cd);
	seq_printf(seq, "Do DMA = %d\n", wh_host_usedma(host));

	return 0;
}

static int wh_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, wh_state_show, inode->i_private);
}

static const struct file_operations wh_fops_state = {
	.owner		= THIS_MODULE,
	.open		= wh_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define DBG_REG(_r) { .addr = WH_SD_##_r, .name = #_r }

struct wh_reg {
	unsigned short	addr;
	unsigned char	*name;
};

static const struct wh_reg debug_regs[] = {
	DBG_REG(ARGU),
	DBG_REG(CMD),
	DBG_REG(STATUS),
	DBG_REG(RSP),
	DBG_REG(CS),
	DBG_REG(BSR),
	DBG_REG(PCR),
	DBG_REG(SWR),
	DBG_REG(TOR),
	DBG_REG(NIS),
	DBG_REG(EIS),
	DBG_REG(NIE),
	DBG_REG(EIE),
	DBG_REG(CAP),
	DBG_REG(CDR),
	DBG_REG(BDBS),
	DBG_REG(DIS),
	DBG_REG(DIE),
	//DBG_REG(BDRX_L),
	//DBG_REG(BDRX_H),
	//DBG_REG(BDTX_L),
	//DBG_REG(BDTX_H),
	DBG_REG(CUSTOM),
	DBG_REG(RSP_2),
	DBG_REG(RSP_3),
	DBG_REG(RSP_4),
	{}
};

static int wh_regs_show(struct seq_file *seq, void *v)
{
	struct wh_host *host = seq->private;
	const struct wh_reg *rptr = debug_regs;

	for (; rptr->name; rptr++)
		seq_printf(seq, "SDI%s\t=0x%08x\n", rptr->name,
			   readl(host->base + rptr->addr));


	return 0;
}

static int wh_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, wh_regs_show, inode->i_private);
}

static const struct file_operations wh_fops_regs = {
	.owner		= THIS_MODULE,
	.open		= wh_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void wh_debugfs_attach(struct wh_host *host)
{
	struct device *dev = &host->pdev->dev;

	host->debug_root = debugfs_create_dir(dev_name(dev), NULL);
	if (IS_ERR(host->debug_root)) {
		dev_err(dev, "failed to create debugfs root\n");
		return;
	}

	host->debug_state = debugfs_create_file("state", 0444,
						host->debug_root, host,
						&wh_fops_state);

	if (IS_ERR(host->debug_state))
		dev_err(dev, "failed to create debug state file\n");

	host->debug_regs = debugfs_create_file("regs", 0444,
					       host->debug_root, host,
					       &wh_fops_regs);

	if (IS_ERR(host->debug_regs))
		dev_err(dev, "failed to create debug regs file\n");
}

static void wh_debugfs_remove(struct wh_host *host)
{
	debugfs_remove(host->debug_regs);
	debugfs_remove(host->debug_state);
	debugfs_remove(host->debug_root);
}

#else
static inline void wh_debugfs_attach(struct wh_host *host) { }
static inline void wh_debugfs_remove(struct wh_host *host) { }

#endif /* CONFIG_DEBUG_FS */

static int wh_probe_pdata(struct wh_host *host)
{
	struct platform_device *pdev = host->pdev;
	struct mmc_host *mmc = host->mmc;
	struct wh_mci_pdata *pdata;

#ifdef ENTER_PRINT
	printk("Enter wh_probe_pdata");
#endif

	if (!pdev->dev.platform_data)
		pdev->dev.platform_data = &wh_def_pdata;

	pdata = pdev->dev.platform_data;

	if (pdata->no_wprotect)
		mmc->caps2 |= MMC_CAP2_NO_WRITE_PROTECT;

	if (pdata->no_detect)
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	if (pdata->wprotect_invert)
		mmc->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;

	if (pdata->detect_invert)
		 mmc->caps2 |= MMC_CAP2_CD_ACTIVE_HIGH;


	//test card detect

	//test card wprotect

	return 0;
}

static int wh_probe_dt(struct wh_host *host)
{
	struct platform_device *pdev = host->pdev;
	struct wh_mci_pdata *pdata;
	struct mmc_host *mmc = host->mmc;
	int ret;

#ifdef ENTER_PRINT
	printk("Enter wh_probe_dt");
#endif

	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdev->dev.platform_data = pdata;

	return 0;
}

static int wh_probe(struct platform_device *pdev)
{
	struct wh_host *host;
	struct mmc_host	*mmc;
	int ret;

#ifdef ENTER_PRINT
	printk("Enter wh_probe");
#endif

	mmc = mmc_alloc_host(sizeof(struct wh_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_out;
	}

	host = mmc_priv(mmc);
	host->mmc 	= mmc;
	host->pdev	= pdev;

	if (pdev->dev.of_node)
		ret = wh_probe_dt(host);
	else
		ret = wh_probe_pdata(host);

	if (ret)
		goto probe_free_host;

	host->pdata = pdev->dev.platform_data;

	spin_lock_init(&host->complete_lock);
	tasklet_init(&host->pio_tasklet, pio_tasklet, (unsigned long) host);

	host->clk_div	= 2;

	host->complete_what 	= COMPLETION_NONE;
	host->pio_active 	= XFER_NONE;

	host->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->mem) {
		dev_err(&pdev->dev,
			"failed to get io memory region resource.\n");

		ret = -ENOENT;
		goto probe_free_gpio;
	}

	host->mem = request_mem_region(host->mem->start,
				       resource_size(host->mem), pdev->name);

	if (!host->mem) {
		dev_err(&pdev->dev, "failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_gpio;
	}

	host->base = ioremap(host->mem->start, resource_size(host->mem));
	if (!host->base) {
		dev_err(&pdev->dev, "failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto probe_free_mem_region;
	}

	host->data_buffer_size = 512;
	ret = of_property_read_u32(pdev->dev.of_node, "data-buffer-phyaddr", &host->data_buffer_phyaddr);
	if (ret)
			return ret;

	host->data_buffer_viraddr = ioremap(host->data_buffer_phyaddr, host->data_buffer_size);
	if (!host->data_buffer_viraddr) {
			dev_err(&pdev->dev, "failed to ioremap() io data buffer.\n");
			ret = -EINVAL;
			goto probe_free_mem_region;
		}

	host->irq = platform_get_irq(pdev, 0);
	//host->irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

#ifdef ENTER_PRINT
	printk("WH-MMC host->irq is %d",host->irq);
#endif

	if (host->irq <= 0) {
		dev_err(&pdev->dev, "failed to get interrupt resource.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	if (request_irq(host->irq, wh_irq, 0, DRIVER_NAME, host)) {
		dev_err(&pdev->dev, "failed to request mci interrupt.\n");
		ret = -ENOENT;
		goto probe_iounmap;
	}

	/* We get spurious interrupts even when we have set the IMSK
	 * register to ignore everything, so use disable_irq() to make
	 * ensure we don't lock the system with un-serviceable requests. */

	disable_irq(host->irq);
	host->irq_state = false;

	/* Depending on the dma state, get a DMA channel to use. */
	dcache_reg = ioremap(FLUSH_32, 0x1);

//	host->clk = clk_get(&pdev->dev, "sdi");  //find "clock-names"
//	if (IS_ERR(host->clk)) {
//		dev_err(&pdev->dev, "failed to find clock source.\n");
//		ret = PTR_ERR(host->clk);
//		host->clk = NULL;
//		goto probe_free_dma;
//	}

	ret = clk_prepare_enable(host->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock source.\n");
		goto clk_free;
	}

	host->clk_rate = clk_get_rate(host->clk);

	mmc->ops 	= &wh_ops;
	mmc->ocr_avail	= MMC_VDD_32_33 | MMC_VDD_33_34;
#ifdef CONFIG_MMC_WH_HW_SDIO_IRQ
	mmc->caps	= MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;
#else
	mmc->caps	= MMC_CAP_4_BIT_DATA | MMC_CAP_UHS_SDR25;
#endif
	mmc->f_min 	= host->clk_rate / (host->clk_div * 256);
	mmc->f_max 	= host->clk_rate / host->clk_div;

	if (host->pdata->ocr_avail)
		mmc->ocr_avail = host->pdata->ocr_avail;

	mmc->max_blk_count	= 1;
	mmc->max_blk_size	= 4095;
	mmc->max_req_size	= 1*4095;
	mmc->max_seg_size	= mmc->max_req_size;

	mmc->max_segs		= 128;


	ret = wh_cpufreq_register(host);
	if (ret) {
		dev_err(&pdev->dev, "failed to register cpufreq\n");
		goto free_dmabuf;
	}

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto free_cpufreq;
	}

	wh_debugfs_attach(host);

	platform_set_drvdata(pdev, mmc);
	dev_info(&pdev->dev, "%s - using %s, %s SDIO IRQ\n", mmc_hostname(mmc),
		 wh_host_usedma(host) ? "dma" : "pio",
		 mmc->caps & MMC_CAP_SDIO_IRQ ? "hw" : "sw");

	return 0;

 free_cpufreq:
	wh_cpufreq_deregister(host);

 free_dmabuf:
	clk_disable_unprepare(host->clk);

 clk_free:
	clk_put(host->clk);

 probe_iounmap:
	iounmap(host->base);

 probe_free_mem_region:
	release_mem_region(host->mem->start, resource_size(host->mem));

 probe_free_gpio:
 	 ;

 probe_free_host:
	mmc_free_host(mmc);

 probe_out:
	return ret;
}

static void wh_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct wh_host *host = mmc_priv(mmc);

	if (host->irq_cd >= 0)
		free_irq(host->irq_cd, host);

	wh_debugfs_remove(host);
	wh_cpufreq_deregister(host);
	mmc_remove_host(mmc);
	clk_disable_unprepare(host->clk);
}

static int wh_remove(struct platform_device *pdev)
{
	struct mmc_host		*mmc  = platform_get_drvdata(pdev);
	struct wh_host	*host = mmc_priv(mmc);

	wh_shutdown(pdev);

	clk_put(host->clk);

	tasklet_disable(&host->pio_tasklet);

	if (wh_host_usedma(host))
		dma_release_channel(host->dma);

	free_irq(host->irq, host);

	if (!pdev->dev.of_node)
		;
	iounmap(host->base);
	release_mem_region(host->mem->start, resource_size(host->mem));

	mmc_free_host(mmc);

	iounmap(dcache_reg);

	return 0;
}

static const struct of_device_id wh_dt_match[] = {
	{
		.compatible = "uctechip,wh-mmc",
		.data = (void *)0,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, wh_dt_match);

static const struct platform_device_id wh_driver_ids[] = {
	{
		.name	= "wh-mmc",
		.driver_data	= 0,
	},
	{ }
};

MODULE_DEVICE_TABLE(platform, wh_driver_ids);

static struct platform_driver wh_driver = {
	.driver	= {
		.name	= "wh-mmc",
		.of_match_table = wh_dt_match,
	},
	.id_table	= wh_driver_ids,
	.probe		= wh_probe,
	.remove		= wh_remove,
	.shutdown	= wh_shutdown,
};

module_platform_driver(wh_driver);

MODULE_DESCRIPTION("WH MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("UCTECHIP <info@uctechip.com>");
