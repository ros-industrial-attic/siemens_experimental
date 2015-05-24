/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx_base.h
******************************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraedert weitergegeben  */
/* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         */
/* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      */
/* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       */
/* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN */
/* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*/
/* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN */
/* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND */
/* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   */
/* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  */
/* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   */
/*                                                                           */
/* This Software is Freeware. It is distributed for free. You may copy,      */
/* modify and use it for free as well as distribute it to others. You may    */
/* only distribute it as a whole, unmodified and with all trademarks and     */
/* copyright notices. You may not distribute it commercially (e.g. as a      */
/* Share-/Freeware-Distributor) without the express written permission of    */
/* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  */
/* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND */
/* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          */
/* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         */
/* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               */
/* If you happen to find any faults in it please tell us.                    */
/*                                                                           */
/*****************************************************************************/

#ifndef __KERNEL__
    #error this header is only for kernel side
#endif

#ifndef _CP16XX_BASE_H
#define _CP16XX_BASE_H

#include "os.h"             /* system dependencies */
#include "cp16xx.h"         /* common(to user and kernel spaces) structures and defines */
#include "dprintern.h"      /* common layer for FW and driver */
#include "wd_dpr_msg.h"     /* watchdog structures and defines */
#include "mgt_dpr_msg.h"    /* management structures and defines */

/* VENDOR and PCI IDs for supported cards */
#define PCI_VENDOR_SIEMENS_AG  0x110a
#define PCI_ID_PROTO           0x4033      /* prototype board without any rom */
#define PCI_ID_CP1616_1        0x4026      /* CP1616 */
#define PCI_ID_CP1616_2        0x4036      /* CP1616 */
#define PCI_ID_CP1604_1        0x4038      /* CP1604 */

/* registers related to reset sequence */
#define ASIC_ID_OFFSET        0x2600
#define ASIC_RESET_REQ_OFFSET 0x260c
#define ASIC_RESET_ACK_OFFSET 0x2610

/* this enum describes the "on card" resources, accessible from host */
enum pci_bar {
    PCI_BAR_FIRST = 0,
    PCI_BAR_EMIF = 0,  /* Head register for external Memory interface */
    PCI_BAR_BOOTROM,   /* Internal Bootrom & APB Peripherals. */
    PCI_BAR_RAM,       /* User RAM - Internel RAM */
    PCI_BAR_IRTE,      /* IRTE-Switch */
    PCI_BAR_EMIF_CS0,  /* EMIF CS0 - Bank 0 des Peripherie Interfaces */
    PCI_BAR_DPRAM,     /* Dualport RAM */
    PCI_BAR_MAX
};

#define MAX_CP16XX_DEVICES       2  /* default support for two cards */
#define MAX_APPLICATIONS         16 /* Each watchdog needs one application instance.
                                       Each user application(PNIO or L2) needs one application instance. */
#define MAX_APP_WATCHDOGS        1  /* default only one application watchdog is allowed */

/* l2 specific */
#define MAX_L2ETH_APPLICATIONS   1  /* default only one layer2 application is allowed */
/* #define MAX_L2ETH_DMA_PAGE_ORDER 7  2 ^ 7 = 128 pages * DPR_GET_PAGE_SIZE */
#define MAX_L2ETH_DMA_PAGE_ORDER 6  /* 2 ^ 6 = 64 pages * DPR_GET_PAGE_SIZE */
#define MIN_L2ETH_DMA_PAGE_ORDER 4  /* 2 ^ 4 = 16  pages * DPR_GET_PAGE_SIZE */

#define FREE_ID                  0  /* describes unused element */

/* common portable code uses these return codes for error. 0 - success */
#define E_CP16XX_WRONGUSERID  -1
#define E_CP16XX_WRONGARG     -2
#define E_CP16XX_WRONGIOCTL   -3
#define E_CP16XX_MAXREACHED   -4
#define E_CP16XX_NOPERMISSION -5
#define E_CP16XX_BUSY         -6
#define E_CP16XX_FAULT        -7
#define E_CP16XX_NOMEMORY     -8
#define E_CP16XX_IOFAULT      -9

/* data block waiting for transfer to user application */
struct cp16xx_block {
    char *addr;
    unsigned long length;
    struct cp16xx_block *next;
};

struct cp16xx_channel {
    /* OS specific extension */
    struct DPR_CHANNEL_OS_INFO os_info;

    /* data from which channel will be transferred */
    DPR_CHN_TYPE channel_number;

    /* applications reference */
    unsigned long user_id;

    /* root element of block queue */
    struct cp16xx_block *block_queue_head;

    /* mutext protects block_queue_head */
    DPR_MUTEX wmutex;

    /* back link to struct cp16xx_app_data */
    void *parent;
};

#define IS_VALID_USER_ID(x) ((x)->user_id != FREE_ID)
#define SET_INVALID_USER_ID(x) ((x)->user_id = FREE_ID)

struct cp16xx_app_data {
    /* OS specific extension */
    struct DPR_APP_OS_INFO os_info;

    /* applications reference */
    unsigned long user_id;

    /* describe type of this structure */
	/*
	 * APPLICATION_EXCLUSIVE mutually exclusives
	 * APPLICATION and APPLICATION_EXCLUSIVE
	 */
    enum APP_TYPE { NOT_USED = 0,
                    APPLICATION,
                    APPLICATION_WD,
                    DRIVER_WD,
                    APPLICATION_EXCLUSIVE,
                    DRIVER_INTERNAL,
                    APPLICATION_L2 } type;

    /* array of channels */
    struct cp16xx_channel pools[DPR_CFG_MAX_CHANNEL];

    /* DMA region of PNIO application,
       used only by library compiled with special flags,
       helpful in the first stage of development */
    struct t_dma_range dma_range;

    /* back link to struct cp16xx_card_data */
    void *parent;

    /* semaphore controls data transfer in mgt channel */
    DPR_SEMAPHORE mgt_synch;

    /* data pointer used by transfer over mgt channel */
    void *mgt_synch_data;

    /* used if this structure of type APPLICATION_WD or DRIVER_WD */
    DPR_UINT32 wd_user;
    DPR_UINT32 wd_ref;
    DPR_UINT32 wd_timeout;
    DPRAM_WD  *wd_cell;
    DPR_TIMER *wd_kernel_timer;
    DPR_UINT16 wd_owner;
    char       wd_buf[1024];
	DPR_UINT32 important_fw_msg_reported;
};

/* describes IO memory of card accessible from host */
struct cp16xx_pci_bar {
    DPR_PCI_PHYSICAL_ADDRESS bar_base_physaddr;
    unsigned long bar_size;
    char *bar_ptr;
    DPR_ATOMIC count;
};

/* describes DMA memory region of host */
struct cp16xx_dma_region {
    void *virt;
    DPR_DMA_PHYSICAL_ADDRESS phys;
    unsigned long size;
    DPR_ATOMIC count;
};

/* prototyp of IRT callback */
typedef void (*IRTCALLBACK)(void *);

#undef SWITCH_DIAG_TRACE /* define if switch port diagnostic trace used */

struct cp16xx_card_data {
    /* OS specific extension */
    struct DPR_CARD_OS_INFO os_info;

    /* IO resources of card */
    struct cp16xx_pci_bar bars[PCI_BAR_MAX];

    /* DMA resources of card */
    struct cp16xx_dma_region irt;
    struct cp16xx_dma_region l2;

#ifdef SWITCH_DIAG_TRACE
    struct cp16xx_dma_region trc;
#endif /* SWITCH_DIAG_TRACE */

    /* default watchdog time in ms */
    int watchdog_cycle;

    /* flag, describes that <driver observes FW> watchdog discovers
       FW timeout */
    int firmware_timeouted;

    /* index of this card, 0 based */
    int cp_index;

    /* general IRQ spinlock */
    DPR_SPINLOCK smplock;

    /* IRT related waitqueues */
    DPR_WAIT_QUEUE_IRT startop_irq, opfault_irq, newcycle_irq;

    /* RT related waitqueue, used mostly for test,
       see 16xxtest sample application and CP16XX_IOCWIRQ ioctl */
    DPR_WAIT_QUEUE_RT rt_irq;

    /* array of applications */
    struct cp16xx_app_data apps[MAX_APPLICATIONS];

    /* this spinlock protects array above */
    DPR_SPINLOCK apps_lock;

    /* applications counter */
    DPR_ATOMIC applications_count;

    /* application watchdogs counter */
    DPR_ATOMIC applications_watchdogs;

    /* layer2 aplications counter */
    /* DPR_ATOMIC l2eth_users; */

    /* reference to daemon kernel thread */
    DPR_THREAD_HANDLE daemon_handle;
    /* semaphore controls daemon kernel thread */
    DPR_SEMAPHORE daemon_resume;

    /* variable describes next job for daemon kernel thread */
    DPR_ATOMIC daemon_work;

    /* pointers to applications, registered these IRT callbacks */
    struct cp16xx_app_data *startop_app, *opfault_app, *newcycle_app;

    /* currently valid IRT callbacks */
    IRTCALLBACK startopfct, opfaultfct, newcyclefct;

 #ifdef DBG_COUNTERS
    unsigned long newcyc_irq_count;  /* see cp16xx_linux_irq.c */
    unsigned long rt_irq_count;
    unsigned long irt_irq_count;
 #endif /* DBG_COUNTERS */

    /* this structure includes common infrastructure of communication with FW */
    CpData cp;

    /* the pointer of the network layer structure of this card.
       Only used if CP16XX_COMBINED_DRIVER is defined otherwise this will be NULL */
	DPR_VOID *ndisAdapter;

	/* pointer used to access the network adapter structure */
#ifdef CP16XX_COMBINED_DRIVER
	/* this handle is needed for allocating DMA area under windows */
	DPR_HANDLE *handle;
	/*
	 * need this semaphore because all the l2 operations should start
	 * after the dpr library is started
	 */
	DPR_SEMAPHORE dprstart_done;

	struct cp16xx_app_data *card_wd_app;
#endif
};

/* describes structure pointed by user interface reference */
struct cp16xx_interface {
    struct cp16xx_card_data  *card;
    struct cp16xx_app_data    *app;
    struct cp16xx_channel *channel;
    DPR_CHN_TYPE    channel_number;
};

/* OS independent functions implemented in cp16xx_base.c */
static void cp16xx_init_global_data(void);
static void cp16xx_uninit_global_data(void);
void cp16xx_irq_set_mode(struct cp16xx_card_data *card);
void cp16xx_irq_reset_mask_irt(struct cp16xx_card_data *card);
void cp16xx_irq_reset_mask_rt(struct cp16xx_card_data *card);
void cp16xx_irq_restore_mask_irt(struct cp16xx_card_data *card);
void cp16xx_irq_restore_mask_rt(struct cp16xx_card_data *card);

void cp16xx_irt_startop_cbf_default(void *arg);
void cp16xx_irt_opfault_cbf_default(void *arg);
void cp16xx_irt_newcycle_cbf_default(void *arg);

void cp16xx_dprlib_setup(struct cp16xx_card_data *card);

int  cp16xx_daemon(void * arg);
void cp16xx_daemon_work(void *arg, enum work work);
void cp16xx_daemon_timer(void *arg);

struct cp16xx_card_data *cp16xx_card_ref_get(int i);
int  cp16xx_card_init(struct cp16xx_card_data *card);
void cp16xx_card_uninit(struct cp16xx_card_data *card);

int cp16xx_dma_init(struct cp16xx_card_data *card);
int cp16xx_dma_uninit(struct cp16xx_card_data *card);

int cp16xx_pci_init(struct cp16xx_card_data *card);
int cp16xx_pci_uninit(struct cp16xx_card_data *card);

int cp16xx_dma_baseaddr_set(struct cp16xx_card_data *card);

/* simple block handling implementation,
   create a block in virtual memory, free it,
   insert at the end of queue, remove it from queue */
struct cp16xx_block *BLOCK_create(unsigned long length);
void BLOCK_free(struct cp16xx_block *block);
int  BLOCK_add_tail(struct cp16xx_channel *channel, struct cp16xx_block *block);
struct cp16xx_block *BLOCK_remove_head(struct cp16xx_channel *channel);
#define BLOCK_queue_is_empty(x) (!(x)->block_queue_head)

struct cp16xx_interface *cp16xx_base_open(struct cp16xx_card_data *card, DPR_CHN_TYPE channel_number);
int  cp16xx_base_release(struct cp16xx_interface *interface);
int  cp16xx_base_ioctl(struct cp16xx_interface *interface,
    unsigned int cmd, void *arg, unsigned long size_in, unsigned long size_out);
int cp16xx_base_read(struct cp16xx_interface *interface, const char *buf, int count);
int cp16xx_base_write(struct cp16xx_interface *interface, const char *buf, int count);

/* OS dependent functions MUST be implemented in cp16xx_<your_os>.c
   they are be called from cp16xx_base.c */
/* Bin: Put more functions declaration here because the NDIS
   part needs those. */
int  cp16xx_os_irq_init(struct cp16xx_card_data *card);
void cp16xx_os_irq_uninit(struct cp16xx_card_data *card);
void cp16xx_os_read_complete(struct cp16xx_channel *channel);
void cp16xx_os_read_abort(struct cp16xx_channel *channel);
int cp16xx_os_read_l2(struct cp16xx_interface *interface, char *buf, int count, int *ret);
int  cp16xx_os_get_info(struct cp16xx_card_data *card, char *buf, int buflen);
DPR_INT16 cp16xx_os_reset_begin(struct cp16xx_card_data *card);
DPR_VOID cp16xx_os_reset_end(struct cp16xx_card_data *card);

#endif /* _CP16XX_BASE_H */
