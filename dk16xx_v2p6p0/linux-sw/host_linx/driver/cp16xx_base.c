/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx_base.c
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

#include "cp16xx_base.h" /* common (kernel side only) structures and defines */

/* PREFACE:
   all functions from this file are OS agnostic, and don't need any porting effort */

/* forward declaration of internal watchdog API */
static void cp16xx_wd_timer(void *arg);
static int  cp16xx_wd_trigger(struct cp16xx_app_data *app);
static int  cp16xx_wd_activate(struct cp16xx_app_data *app);
static int  cp16xx_wd_deactivate(struct cp16xx_app_data *app);
static int  cp16xx_wd_deactivate_all_wds(struct cp16xx_card_data *card);
static int  cp16xx_wd_observe(struct cp16xx_app_data *app);

/* static array of all cards pointers */
static struct cp16xx_card_data *cp16xx_cards[MAX_CP16XX_DEVICES];
/* spinlock protects cp16xx_cards array */
static DPR_SPINLOCK cp16xx_cards_lock;

#ifdef DBG_COUNTERS
/* printk card info counter and more for debugging only
 */
void dbg_print_info(struct cp16xx_card_data *card)
{
    #if !defined(KERN_WARNING)
        #define KERN_WARNING " "
    #endif
    printk(KERN_WARNING "cp16xx-dbg: rt_irq_count                    = %d   \n",card->rt_irq_count);
    printk(KERN_WARNING "cp16xx-dbg: irt_irq_count                   = %d   \n",card->irt_irq_count);
    printk(KERN_WARNING "cp16xx-dbg: newcyc_irq_count                = %d   \n",card->newcyc_irq_count);
    return;
}
#endif /* DBG_COUNTERS */

static void cp16xx_init_global_data(void)
{
    memset(cp16xx_cards, 0, sizeof(cp16xx_cards));
    DPR_SPINLOCK_INIT(cp16xx_cards_lock);
}

static void cp16xx_uninit_global_data(void)
{
    DPR_SPINLOCK_UNINIT(cp16xx_cards_lock);
}

/* primitives for simple memory block handling in a fifo queue
   create, destroy, add_to_tail, remove_from_head */
struct cp16xx_block *BLOCK_create(unsigned long length)
{
    struct cp16xx_block *block;
    void *addr = DPR_VMALLOC(length);
    if ( !addr )
        return NULL;

    block = DPR_VMALLOC(sizeof(struct cp16xx_block));
    if ( !block ) {
        DPR_VFREE(addr);
        return NULL;
    }

    block->next = NULL;
    block->addr = addr;
    block->length = length;

    return block;
}

void BLOCK_free(struct cp16xx_block *block)
{
    if ( !block )
        return;

    if ( block->addr )
        DPR_VFREE(block->addr);

    DPR_VFREE(block);
}

int BLOCK_add_tail(struct cp16xx_channel *channel, struct cp16xx_block *block)
{
    struct cp16xx_block *tail;
    int ret = 0;

    if ( !channel || !block )
        return ret;

    DPR_CHANNEL_LOCK(channel);

    if ( !IS_VALID_USER_ID(channel) )
        goto BLOCK_add_tail_unlock_exit;

    if ( !channel->block_queue_head ) {
        channel->block_queue_head = block;
    } else {
        for ( tail = channel->block_queue_head;
            tail->next;
            tail = tail->next )
            ;
        tail->next = block;
    }
    ret = 1;

    BLOCK_add_tail_unlock_exit:
    DPR_CHANNEL_UNLOCK(channel);

    return ret;
}

struct cp16xx_block *BLOCK_remove_head(struct cp16xx_channel *channel)
{
    struct cp16xx_block *block;
    if ( !channel )
        return NULL;

    DPR_CHANNEL_LOCK(channel);
    if ( !channel->block_queue_head ) {
        block = NULL;
    } else {
        block = channel->block_queue_head;
        channel->block_queue_head = block->next;
    }
    DPR_CHANNEL_UNLOCK(channel);

    return block;
}

/* default timeout driver watchdog in ms
 set it to zero to deactivate watchdog for debug purposes */
#if defined(RTAI)
  int watchdog_cycle = 0; /* driver watchdog (firmware/host only) is disabled if RTAI defined */
#else
  int watchdog_cycle = 1000; /* millisec. must be set of 1000 presumably because of NDIS (was 200) */
#endif

/* IRT related functions
   cp16xx_irt_[startop|opfault|newcycle]_cbf are plugable callbacks for IRQs
   cp16xx_irt_[startop|opfault|newcycle]_cbf_default are standard callbacks
   only cp16xx_startop_default is not empty.

   cp16xx_irt_set_opdone() is a helper */

inline void cp16xx_irt_set_opdone(struct cp16xx_card_data *card)
{
    DPR_WRITE_UINT32(DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + IRT_SYNCMODE) | 0x00000003,
                     card->bars[PCI_BAR_IRTE].bar_ptr + IRT_SYNCMODE);
}

void cp16xx_irt_startop_cbf_default(void *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    cp16xx_irt_set_opdone(card);
}

static void cp16xx_irt_startop_cbf(void *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    DPR_WAKEUP_QUEUE_IRT(card->startop_irq);
}

void cp16xx_irt_opfault_cbf_default(void *arg)
{
}

static void cp16xx_irt_opfault_cbf(void *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    DPR_WAKEUP_QUEUE_IRT(card->opfault_irq);
}

void cp16xx_irt_newcycle_cbf_default(void *arg)
{
}

static void cp16xx_irt_newcycle_cbf(void *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    DPR_WAKEUP_QUEUE_IRT(card->newcycle_irq);
}

/* IRQ related helper. Manipulate IRQ controller registers */
void cp16xx_irq_reset_mask_irt(struct cp16xx_card_data *card)
{
    DPR_WRITE_UINT32(0x00000000, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_MASK_IRT);
}

void cp16xx_irq_reset_mask_rt(struct cp16xx_card_data *card)
{
    DPR_WRITE_UINT32(0x00000000, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_MASK_RT);
}

void cp16xx_irq_set_mode(struct cp16xx_card_data *card)
{
    DPR_WRITE_UINT32(0x00000000, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ_MODE);
}

void cp16xx_irq_restore_mask_irt(struct cp16xx_card_data *card)
{
    DPR_UINT32 bits = BIT_IRQ_STARTOP | OPFAULT_BITS;
    if ( card->newcycle_app )
        bits |= BIT_IRQ_NEWCYCLE;
    DPR_WRITE_UINT32(bits, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_MASK_IRT);
}

void cp16xx_irq_restore_mask_rt(struct cp16xx_card_data *card)
{
    DPR_WRITE_UINT32(BIT_IRQ_HP, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_MASK_RT);
}

static void cp16xx_irq_trigger(void *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    DPR_WRITE_UINT32(0x00000001, card->bars[PCI_BAR_IRTE].bar_ptr + SP_IRQ_ACTIVATE);
}

/* handling of application references
   card->apps array protected under card->apps_lock */
static struct cp16xx_app_data *cp16xx_app_search(struct cp16xx_card_data *card, unsigned long user_id)
{
    DPR_SPINLOCK_FLAGS flags;
    struct cp16xx_app_data *app = NULL;

    DPRLIBCHATMSG("begin: usr-id=%lu MAX-APPs=%d \n", user_id, MAX_APPLICATIONS);

    if ( user_id > 0 && user_id <= MAX_APPLICATIONS ) {
        DPR_SPINLOCK_LOCK(card->apps_lock, flags);
        app = &card->apps[user_id - 1];
        if ( app->user_id != user_id )
            app = NULL;
        DPR_SPINLOCK_UNLOCK(card->apps_lock, flags);
    }

    DPRLIBCHATMSG("end: usr-id=%lu app=%#x \n", user_id, app);
    return app;
}

static struct cp16xx_app_data *cp16xx_app_new(struct cp16xx_card_data *card, enum APP_TYPE type)
{
    DPR_SPINLOCK_FLAGS flags;
    struct cp16xx_app_data *app = NULL;
    static int lastindex = 0;
    int i, tmp_app_cnt = 0;

    DPRLIBLOGMSG("begin type=%d (1=APP,2=APPWD,3=DRVWD,4=APPEXL,5=DRVINT,6=APPL2)\n", type);

    DPR_SPINLOCK_LOCK(card->apps_lock, flags);

    if ( DPR_ATOMIC_READ(card->applications_count) >= MAX_APPLICATIONS ) {
        DPRLIBERRMSG("too much applications: max=%d \n", MAX_APPLICATIONS);
        goto cp16xx_app_new_exit;
    }
    else {
        /* check, is allready one APPLICATION_EXCLUSIVE registered */
        if ( type == APPLICATION || type == APPLICATION_EXCLUSIVE ) {
            for ( i = 0; i < MAX_APPLICATIONS; ++i ) {
                if ( card->apps[i].user_id != FREE_ID &&
                     card->apps[i].type == APPLICATION_EXCLUSIVE ) {
                    DPRLIBERRMSG("one APPLICATION_EXCLUSIVE allready registered, \
                    no other application of type APPLICATION or \
                    APPLICATION_EXCLUSIVE can be registered yet\n");
                    goto cp16xx_app_new_exit;
                }
            }
        }
    }

    if ( type == APPLICATION_EXCLUSIVE ) {
        /* needs exclusive access to CP */

        /* count all APPLICATION & APPLICATION_EXCLUSIVE */
        for ( i = 0; i < MAX_APPLICATIONS; ++i ) {
            if ( card->apps[i].user_id != FREE_ID &&
                 ( (card->apps[i].type == APPLICATION_EXCLUSIVE) | (card->apps[i].type == APPLICATION) ) ) {
                tmp_app_cnt++;
            }
        }

        if ( tmp_app_cnt == 0 ) {
            /* allow EXCLUSIVE open, if no or only L2 application registered */
            DPRLIBLOGMSG("APPLICATION_EXCLUSIVE allowed\n");
        }
        else {
            DPRLIBERRMSG("less than one application registered, no exclusive access possible app_cnt(APP & APP_EXCLUSIVE)=%d\n",
                         tmp_app_cnt);
            goto cp16xx_app_new_exit;
        }
    } else if ( type == APPLICATION_L2 ) {
        /* count all APPLICATION_L2 */
        for ( i = 0; i < MAX_APPLICATIONS; ++i ) {
            if ( card->apps[i].user_id != FREE_ID &&
                 ( card->apps[i].type == APPLICATION_L2) ) {
                tmp_app_cnt++;
            }
        }

        if ( tmp_app_cnt + 1 > MAX_L2ETH_APPLICATIONS ) {
            DPRLIBERRMSG("too much L2 applicatins\n");
            goto cp16xx_app_new_exit;
        }
    }

    if ( type == APPLICATION_WD &&
        DPR_ATOMIC_READ(card->applications_watchdogs) >= MAX_APP_WATCHDOGS ) {
        DPRLIBERRMSG("too much appl. watchdogs: max=%d\n", MAX_APP_WATCHDOGS);
        goto cp16xx_app_new_exit;
    }

    /* we try to avoid the same applications ID in sequencial sessions */
    for ( i = lastindex + 1; i < MAX_APPLICATIONS; ++i ) {
        app = &(card->apps[i]);
        if ( app->user_id == FREE_ID ) {
            break;
        }
        app = NULL;
    }
    /* ok, last applications ID is used, search from begin */
    if ( !app ) {
        for ( i = 0; i < MAX_APPLICATIONS; ++i ) {
            app = &(card->apps[i]);
            if ( app->user_id == FREE_ID ) {
                break;
            }
            app = NULL;
        }
    }

    if ( app ) {
        app->user_id = i + 1;
        lastindex = i;
        DPRLIBLOGMSG("new app: type=%d user_id=%lu\n", type, app->user_id);
        app->type = type;
        app->parent = card;
        DPR_ATOMIC_INC(card->applications_count);
        if ( type == APPLICATION_WD ) {
            DPR_ATOMIC_INC(card->applications_watchdogs);
            DPRLIBLOGMSG("new app: type=%d=APPLICATION_WD: current cnt=%lu\n",
                          APPLICATION_WD, DPR_ATOMIC_READ(card->applications_watchdogs));
        }
    }

  #ifdef DBG_COUNTERS
    card->newcyc_irq_count = 0;
    card->rt_irq_count = 0;
    card->irt_irq_count = 0;
  #endif /* DBG_COUNTERS */

    cp16xx_app_new_exit:
    DPR_SPINLOCK_UNLOCK(card->apps_lock, flags);

    DPRLIBLOGMSG("end: ret=%#x (0=ERR)\n", app);
    return app;
}

static int cp16xx_app_free(struct cp16xx_card_data *card, struct cp16xx_app_data *app)
{
    DPR_SPINLOCK_FLAGS flags;
    int ret = 0;

    DPRLIBLOGMSG("begin: app=%#x \n",app);
    if (!app ) {
        DPRLIBERRMSG("ERR !app\n");
        return -1;
    }

    if ( !(IS_VALID_USER_ID(app)) ) {
        DPRLIBERRMSG("ERR user-id\n");
        return -1;
    }

    DPR_SPINLOCK_LOCK(card->apps_lock, flags);

    DPRLIBLOGMSG("app type=%d user_id=%lu\n", app->type, app->user_id);
    SET_INVALID_USER_ID(app);
    app->important_fw_msg_reported = 0;

  #ifdef DBG_COUNTERS
    dbg_print_info(card);
  #endif /* DBG_COUNTERS */

    DPR_ATOMIC_DEC(card->applications_count);
    DPRLIBLOGMSG("current appl. cnt=%lu\n", DPR_ATOMIC_READ(card->applications_count));

    if ( app->type == APPLICATION_WD ) {
        DPR_ATOMIC_DEC(card->applications_watchdogs);
        DPRLIBLOGMSG("type: APPLICATION_WD current cnt=%lu\n",DPR_ATOMIC_READ(card->applications_watchdogs));
    }
    DPR_WAKEUP_QUEUE(card->rt_irq);

    if ( card->startop_app == app ) {
        card->startopfct = cp16xx_irt_startop_cbf_default;
        card->startop_app = NULL;
        cp16xx_irt_startop_cbf(card);
    }
    if ( card->opfault_app == app ) {
        card->opfaultfct = cp16xx_irt_opfault_cbf_default;
        card->opfault_app = NULL;
        cp16xx_irt_opfault_cbf(card);
    }
    if ( card->newcycle_app == app ) {
        card->newcyclefct = cp16xx_irt_newcycle_cbf_default;
        card->newcycle_app = NULL;
        cp16xx_irq_restore_mask_irt(card);
        cp16xx_irt_newcycle_cbf(card);
    }
    app->type = NOT_USED;
    app->parent = NULL;

    DPR_SPINLOCK_UNLOCK(card->apps_lock, flags);

    DPRLIBLOGMSG("end: ret=%d\n", ret);  // always ok
    return ret;
}

#ifdef SWITCH_DIAG_TRACE

/* diagnose helper, used in cp16xxtest sample application */
static int cp16xx_get_switch_diag_trc(struct cp16xx_card_data *card, struct t_switch_diag_trc *usr_trc)
{
    unsigned long l,k;
    unsigned long *pTCW = (unsigned long *)card->trc.virt;
    usr_trc->buf_len = 0;

    #define TRC_LIMIT 40

    /* ---------------  show state of trace modules ------------*/
    DPRLIBINFMSG("HP_TRACE_COMMAND= %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_COMMAND));
    DPRLIBINFMSG("HP_TRACE_MODE = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_MODE));
    DPRLIBINFMSG("HP_TRACE_BASE_ADDR = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_BASE_ADDR));
    DPRLIBINFMSG("HP_TRACE_BUF_LEN = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_BUF_LEN));
    DPR_WRITE_UINT32(TRC_LIMIT, card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_TRANS_LIM);
    DPRLIBINFMSG("HP_TRACE_TRANS_LIM = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_TRANS_LIM));
    DPRLIBINFMSG("HP_TRACE_BUF_ENTRY = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_BUF_ENTRY));

    /*----------------- prepare TCW-Command --------------------*/
    pTCW[0] = 0x3C00; /* 60 entries for a trace exist, in realy  64, but during alignment 64/16 = 4 entries are lost */
    pTCW[1] = 0xC0000000 + DPR_PHYSICAL_ADDRESS_TO_ULONG(card->trc.phys) + 0x4 * 2; /* set addres of DMA buffer */
    pTCW[1] = (pTCW[1] + 7) & ~0x7; /* align on 64 bit bound */

    /*----------------- set filter -------*/
    DPR_WRITE_UINT32(usr_trc->port_filter[0] & 0x1FFFF,
                     card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P0);
    DPR_WRITE_UINT32(usr_trc->port_filter[1] & 0x1FFFF,
                     card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P1);
    DPR_WRITE_UINT32(usr_trc->port_filter[2] & 0x1FFFF,
                     card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P2);
    DPR_WRITE_UINT32(usr_trc->port_filter[3] & 0x1FFFF,
                     card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P3);
    DPRLIBINFMSG("HP_TRACE_DIAG_ENABLE_P0,P1,P2,P3 = %x, %x, %x, %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P0),
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P1),
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P2),
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_DIAG_ENABLE_P3));

    /* set MODE */
    DPR_WRITE_UINT32(0x1, /* reset */
                     card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_MODE);

    /* set COMMAND - TCW */
    DPR_WRITE_UINT32(0xC0000000 + DPR_PHYSICAL_ADDRESS_TO_ULONG(card->trc.phys),
                     (card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_COMMAND));

    /* start */
    DPR_WRITE_UINT32(0x4, card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_MODE);

    DPRLIBINFMSG("after set HP_TRACE_COMMAND= %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_COMMAND));

    l = k=0;
    while ( (l < TRC_LIMIT) && (k < 0xffffff) ) {
        l = DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_BUF_ENTRY);  /* to spend time */
        ++k;
    }
    DPR_WRITE_UINT32(0x2, /* stoppen */
                     card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_MODE);

    DPRLIBINFMSG("trace buffer was filled\n");
    DPRLIBINFMSG("after set HP_TRACE_MODE -> COMMAND pTCW[0],[1] = %lx,%lx\n", pTCW[0], pTCW[1]);

    /* set MODE */
    /* while((pTCW[0]&0x80)==0) */
    {
        if ( !(pTCW[0] & 0x80) ) {
            DPRLIBINFMSG("set execute tcw\n");
            DPR_WRITE_UINT32( 0x100 , /* execute tcw */
                              card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_MODE);

            for ( k = 0; k < 0xffffff && (pTCW[0]&0x80) == 0; ++k )
                l = DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_COMMAND);  /* to spend time */
        }
    }

    usr_trc->TCW_0 = pTCW[0];
    usr_trc->buf_len = ((pTCW[0]  >> 8) & 0xFFFF) * 16;

    DPRLIBINFMSG("after set HP_TRACE_MODE -> HP_TRACE_COMMAND= %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_COMMAND));

    DPRLIBINFMSG("after set HP_TRACE_MODE -> HP_TRACE_BUF_ENTRY = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_BUF_ENTRY));

    DPRLIBINFMSG("after set HP_TRACE_MODE -> COMMAND pTCW[0],[1] = %lx,%lx\n", pTCW[0], pTCW[1]);

    DPRLIBINFMSG("after set HP_TRACE_MODE -> HP_TRACE_MODE = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_MODE));
    DPRLIBINFMSG("after set HP_TRACE_MODE -> HP_TRACE_FAULT_CNT = %x\n",
                 DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_TRACE_FAULT_CNT));

    return 0;
}

#endif /* SWITCH_DIAG_TRACE */

/* cp16xx_base(open|release|ioctl|read|write) provide common service for driver.

   cp16xx_base_open - is very simple, initializes common structure for instance reference

   cp16xx_base_release - notificates FW about disappering of application instance over control channel,
                         flush block queues for another channel, releases channel waitqueue.
                         for both it frees common structure.

   cp16xx_base_ioctl - is very simple demultiplexor
   cp16xx_base_ioctl2 - serves IOCTL over control channels
   cp16xx_base_ioctl1 - serves IOCTL over all another channels

   cp16xx_base_read - does only check access, the read will be done in cp16xx_os_read.

   cp16xx_base_write - does complete write */
struct cp16xx_interface *cp16xx_base_open(struct cp16xx_card_data *card, DPR_CHN_TYPE channel_number)
{
    struct cp16xx_interface *interface;

    interface = (struct cp16xx_interface *)DPR_ZALLOC(sizeof(struct cp16xx_interface));

    DPRLIBLOGMSG("-> base_open chnl-num=%d\n", channel_number);

    if ( interface ) {
        interface->card = card;
        interface->app = NULL;
        interface->channel = NULL;
        interface->channel_number = channel_number;
    }

    DPRLIBLOGMSG("<- base_open chnl-num=%d ifc=%#x \n", channel_number, interface);
    return interface;
}

int cp16xx_base_release(struct cp16xx_interface *interface)
{
    struct cp16xx_block *block;

    DPRLIBLOGMSG("begin\n");

    DPR_ASSERT(interface && interface->card);

    if ( interface->channel_number == CONTROL_INTERFACE ) {

        DPRLIBLOGMSG("base_release: CONTROL_INTERFACE chnl-num=%d \n",interface->channel_number);

        if ( !interface->app )
            goto cp16xx_base_release_exit;

        if ( IS_VALID_USER_ID(interface->app) ) {
            /* emeregence close */
            DPRMGTCHNL_TO_MGT_RB rqb;

            rqb.header.hostref = CPU_TO_LE(interface->app->user_id);
            rqb.header.subsystem = FW_SUBSYS_CP_MGT;
            rqb.header.userid = CPU_TO_LE(0x55667788);
            rqb.header.response = CPU_TO_LE(0);
            rqb.header.userdatalength = CPU_TO_LE(sizeof(rqb.mgt_opcode));
            rqb.mgt_opcode = CPU_TO_LE16(OPC_MGT_HOST_ABORT_REQ);

            interface->app->mgt_synch_data = &rqb;

            DPRLIBLOGMSG("send abort for application_id %lu\n", interface->app->user_id);

            if ( DPR_OK != DPRLIB_channel_write_message(&interface->card->cp,
                                                        DPR_MGT_CHN, 0, &rqb, sizeof(rqb)) ) {
                interface->app->mgt_synch_data = NULL;
                DPRLIBERRMSG("fail to send over mgt channel\n");
                goto cp16xx_control_release_nowait;
            }

            if ( DPR_SEM_WAIT_FOR_WAKEUP(interface->app->mgt_synch, 1000) ) {
                interface->app->mgt_synch_data = NULL;
                DPRLIBLOGMSG("got a signal or timeout\n");
                goto cp16xx_control_release_nowait;
            }

            DPR_ASSERT(LE_TO_CPU16(rqb.mgt_opcode) == OPC_MGT_HOST_ABORT_CNF);
            if ( rqb.header.response ) {
                DPRLIBERRMSG("got an invalid response 0x%x\n", LE_TO_CPU(rqb.header.response));
            } else {
                DPRLIBLOGMSG("got a confirmation for abort of application_id %lu\n", interface->app->user_id);
            }

            cp16xx_control_release_nowait:
            cp16xx_app_free(interface->card, interface->app);
        }
    }
    else {
        DPRLIBLOGMSG("base_release: CHNL-xxx chnl-num=%d \n",interface->channel_number);
        if ( !interface->channel )
            goto cp16xx_base_release_exit;

        if ( interface->app && interface->app->wd_user == WD_USER_REF_HOSTAPP_MIN )
            cp16xx_wd_deactivate(interface->app);

        DPR_CHANNEL_LOCK(interface->channel);
        /* mark channel as unused, everybody stops to add new blocks now */
        SET_INVALID_USER_ID(interface->channel);
        DPR_CHANNEL_UNLOCK(interface->channel);

        /* free all outstanding blocks */
        while ( (block = BLOCK_remove_head(interface->channel)) )
            BLOCK_free(block);

        /* check if it is layer2 channel binding
        if(DPR_L2_SEND_CHN == interface->channel_number ||
            DPR_L2_RECV_CHN == interface->channel_number) {
            if(0 < DPR_ATOMIC_READ(interface->card->l2eth_users)) {
                DPR_ATOMIC_DEC(interface->card->l2eth_users);
                DPRLIBLOGMSG("Layer2 channel no %d, l2eth_users_count %d \n",
                    interface->channel_number, DPR_ATOMIC_READ(interface->card->l2eth_users));
            }
        }*/

        cp16xx_os_read_abort(interface->channel);
    }

    cp16xx_base_release_exit:
    DPR_FREE(interface);

    DPRLIBLOGMSG("end\n");

    return 0;
}

int cp16xx_base_ioctl_1(struct cp16xx_interface *interface,
                        unsigned int cmd, void *arg, unsigned long size_in, unsigned long size_out)
{
    struct cp16xx_app_data *app_wd;
    struct t_read_pool *user_pool;
    struct t_register_appl_wd *user_appl_wd;
    struct cp16xx_block *block;
    RING * p_ring = 0;
    int ret = 0;

    DPR_ASSERT(interface && interface->card);

    DPRLIBLOGMSG("begin: cmd=%#x size-in=%d size-out=%d \n", cmd, size_in, size_out );

    switch ( cmd ) {
        case CP16XX_IOC_OWD:
        case CP16XX_IOC_CWD:
        case CP16XX_IOC_TWD:
            if ( DPR_MGT_CHN != interface->channel_number )
                return E_CP16XX_WRONGIOCTL;
            break;
    };

    switch ( cmd ) {
        case CP16XX_IOC_BIND:
            DPR_ASSERT(!interface->app && !interface->channel);

            DPRLIBLOGMSG(">CP16XX_IOC_BIND: usr-id=%d ch=%d pool=%#x \n",
                        ((struct t_read_pool *)arg)->user_id, interface->channel_number, arg );

            if ( size_in != sizeof(struct t_read_pool) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_read_pool));
                return E_CP16XX_WRONGARG;
            }

            user_pool = (struct t_read_pool *)arg;
            ret = sizeof(struct t_read_pool);

            interface->app = cp16xx_app_search(interface->card, user_pool->user_id);
            if ( !interface->app ) /* no such application */
                return E_CP16XX_WRONGUSERID;

            /* check if it is layer2 channel binding */
            if ( DPR_L2_SEND_CHN == interface->channel_number ||
                 DPR_L2_RECV_CHN == interface->channel_number ) {
                /*if((MAX_L2ETH_APPLICATIONS * 2) <= DPR_ATOMIC_READ(interface->card->l2eth_users)) {
                    // two l2-channels per l2-application
                    // to much layer2 applications
                    DPRLIBERRMSG("Layer2 channel no %d, too much layer2 applications now %u\n",
                        interface->channel_number, DPR_ATOMIC_READ(interface->card->l2eth_users));
                    return E_CP16XX_MAXREACHED;
                } else {
                    DPR_ATOMIC_INC(interface->card->l2eth_users);    // increase use count
                    DPRLIBLOGMSG("Layer2 channel no %d, l2eth_users_count %d\n",
                        interface->channel_number, DPR_ATOMIC_READ(interface->card->l2eth_users));
                }*/
                if ( interface->app->type != APPLICATION_L2 ) {
                    DPRLIBERRMSG("only application from type <APPLICATION_L2> can bind L2_SEND/RECV_CHN\n");
                    interface->app = NULL;
                    return E_CP16XX_NOPERMISSION;
                }
            }

            interface->channel = &interface->app->pools[interface->channel_number];
            DPR_ASSERT(interface->channel->channel_number == interface->channel_number);
            DPR_ASSERT(interface->channel->block_queue_head == NULL);

            DPR_CHANNEL_LOCK(interface->channel);
            interface->channel->parent = interface->app;
            interface->channel->user_id = interface->app->user_id;

            p_ring = &interface->card->cp.pDpramConfigArea->Rings[DPRLIB_READ_RING(interface->channel_number)];
            user_pool->read_length = DPR_READ_UINT32(&p_ring->pRingEnd) - DPR_READ_UINT32(&p_ring->pRingStart);
            p_ring = &interface->card->cp.pDpramConfigArea->Rings[DPRLIB_WRITE_RING(interface->channel_number)];
            user_pool->write_length = DPR_READ_UINT32(&p_ring->pRingEnd) - DPR_READ_UINT32(&p_ring->pRingStart);

           /* reset countable sema. Sema could be >0 (available) because "UNBIND" increments it to release
            * possibly waiting user receive thread. Sama have to be initially "unavailable" (cleared).
            * See UNBIND cp16xx_os_read_abort()
            */
            DPR_CHANNEL_RESET_SEMA(interface->channel);

            DPR_CHANNEL_UNLOCK(interface->channel);

            DPRLIBLOGMSG("<CP16XX_IOC_BIND: usr-id=%d app-usr-id=%d \n",
	                     ((struct t_read_pool *)arg)->user_id, interface->app->user_id);
            break;

        case CP16XX_IOC_UNBIND:
            DPR_ASSERT(interface->app && interface->channel);

            DPRLIBLOGMSG(">CP16XX_IOC_UNBIND: usr-id=%d ch=%d pool=%#x \n",
                        ((struct t_read_pool *)arg)->user_id, interface->channel_number, arg );

            if ( size_in != sizeof(struct t_read_pool) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_read_pool));
                return E_CP16XX_WRONGARG;
            }

            user_pool = (struct t_read_pool *)arg;

            if ( interface->channel->user_id != user_pool->user_id ) {
                DPRLIBERRMSG("WRONG ifc-usr-id=%lu NEQ pool-usr-id=%lu \n", interface->channel->user_id, user_pool->user_id );
                return E_CP16XX_WRONGUSERID;
            }

            if ( interface->app->user_id != interface->channel->user_id ) {
                DPRLIBERRMSG("WRONG app-usr-id=%lu NEQ chnl-usr-id=%lu \n", interface->app->user_id, interface->channel->user_id );
                return E_CP16XX_WRONGUSERID;
            }

            DPR_CHANNEL_LOCK(interface->channel);
            /* mark channel as unused, everybody stops to add new blocks now */
            SET_INVALID_USER_ID(interface->channel);
            DPR_CHANNEL_UNLOCK(interface->channel);

            /* free all outstanding blocks */
            while ( (block = BLOCK_remove_head(interface->channel)) )
                BLOCK_free(block);

            cp16xx_os_read_abort(interface->channel);

            /*interface->channel = NULL;*/
            DPRLIBLOGMSG("<CP16XX_IOC_UNBIND: usr-id=%d \n", ((struct t_read_pool *)arg)->user_id );

            break;

        case CP16XX_IOC_OWD: /* register watchdog without application binding, need
                                new application on its own */

            if ( size_out != sizeof(struct t_register_appl_wd) ) {
                DPRLIBERRMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_register_appl_wd));
                return E_CP16XX_WRONGARG;
            }

            user_appl_wd = (struct t_register_appl_wd *)arg;
            ret = sizeof(struct t_register_appl_wd);

            app_wd = cp16xx_app_new(interface->card, APPLICATION_WD);
            if ( !app_wd )
                return E_CP16XX_MAXREACHED;

            app_wd->wd_timeout = user_appl_wd->timeout;
            app_wd->wd_owner = WD_FLAG_FIRMWARE;
            app_wd->wd_user = WD_USER_REF_HOSTAPP_MIN; /* ??? */

            interface->app = app_wd;
            interface->channel = &interface->app->pools[DPR_MGT_CHN];
            DPR_ASSERT(interface->channel->block_queue_head == NULL);
            DPR_CHANNEL_LOCK(interface->channel);
            interface->channel->parent = interface->app;
            interface->channel->user_id = interface->app->user_id;
            DPR_CHANNEL_UNLOCK(interface->channel);

            if ( cp16xx_wd_activate(app_wd) ) {
                cp16xx_app_free(interface->card, app_wd);
                return E_CP16XX_FAULT;
            }

            user_appl_wd->wd_id = app_wd->wd_ref;
            user_appl_wd->user_id = app_wd->user_id;

            p_ring = &interface->card->cp.pDpramConfigArea->Rings[DPRLIB_READ_RING(interface->channel->channel_number)];
            user_appl_wd->read_length = DPR_READ_UINT32(&p_ring->pRingEnd) - DPR_READ_UINT32(&p_ring->pRingStart);

            break;

        case CP16XX_IOC_CWD: /* unregister watchdog */

            if ( size_in != sizeof(struct t_register_appl_wd) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_register_appl_wd));
                return E_CP16XX_WRONGARG;
            }

            user_appl_wd = (struct t_register_appl_wd *)arg;

            app_wd = cp16xx_app_search(interface->card, user_appl_wd->user_id);
            if ( !app_wd )
                return E_CP16XX_WRONGUSERID;


            app_wd->wd_timeout = 0;
            ret = cp16xx_wd_deactivate(app_wd);

            /* kick up, the waiting library thread that read from mgt channel, to be terminated */
            cp16xx_os_read_abort(interface->channel);
            break;

        case CP16XX_IOC_TWD:

            if ( size_in != sizeof(struct t_register_appl_wd) ) {
                DPRLIBERRMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_register_appl_wd));
                return E_CP16XX_WRONGARG;
            }

            user_appl_wd = (struct t_register_appl_wd *)arg;

            app_wd = cp16xx_app_search(interface->card, user_appl_wd->user_id);
            if ( !app_wd )
                return E_CP16XX_WRONGUSERID;

            ret = cp16xx_wd_trigger(app_wd);
            break;
		// new read write over ioctl to support pniolib in usermode
		case CP16XX_WRITEF:
			DPRLIBLOGMSG("begin, CP16XX_WRITEF: card %d channel_number %d user_id %d\n",interface->card, 0 /* interface->channel_number*/,interface->app->user_id );
			DPRLIBLOGMSG("DPRLIB_channel_write_message channel_number = %x user_id = %x count = %d\n",interface->channel_number,interface->app->user_id,size_in);
			ret = DPRLIB_channel_write_message(&interface->card->cp,
										  interface->channel_number , interface->app->user_id, (void *)arg, size_in);
            break;

        default:
            DPRLIBERRMSG("unkown cmd %u \n", cmd );
            ret = E_CP16XX_WRONGIOCTL;
    }

    DPRLIBLOGMSG("end: cmd=%#x \n", cmd );
    return ret;
}

int cp16xx_base_ioctl_2(struct cp16xx_interface *interface,
                        unsigned int cmd, void *arg, unsigned long size_in, unsigned long size_out)
{
    struct t_register_app *user_struct_app;
    struct t_dma_range *user_dma_range;
    struct t_rw_direct *user_struct_direct;
    struct t_dma_address *user_struct_dma_address;
    struct t_read_timer *tmp_timer;
    struct t_read_pool *user_pool;

    char *tmp = NULL;
    void *buffer;
    int i, ret = 0;
    DPR_SPINLOCK_FLAGS flags;
    enum APP_TYPE      open_app_type;

#ifdef SWITCH_DIAG_TRACE
    struct t_switch_diag_trc *user_switch_diag_trc;
#endif /* SWITCH_DIAG_TRACE */

    switch ( cmd ) {
        case CP16XX_IOCSHUTDOWN: DPRLIBLOGMSG("begin, cmd CP16XX_IOCSHUTDOWN\n"); break;
        case CP16XX_IOCRESET:    DPRLIBLOGMSG("begin, cmd CP16XX_IOCRESET\n"); break;
        case CP16XX_IOCREGR:     DPRLIBLOGMSG("begin, cmd CP16XX_IOCREGR\n"); break;
        case CP16XX_IOCREGW:     DPRLIBLOGMSG("begin, cmd CP16XX_IOCREGW\n"); break;
        case CP16XX_IOCDPRAMR:   DPRLIBLOGMSG("begin, cmd CP16XX_IOCDPRAMR\n"); break;
        case CP16XX_IOCDPRAMW:   DPRLIBLOGMSG("begin, cmd CP16XX_IOCDPRAMW\n"); break;
        case CP16XX_IOCWIRQ:     DPRLIBLOGMSG("begin, cmd CP16XX_IOCWIRQ\n"); break;
        case CP16XX_IOCDMAR:     DPRLIBLOGMSG("begin, cmd CP16XX_IOCDMAR\n"); break;
        case CP16XX_IOCDMAW:     DPRLIBLOGMSG("begin, cmd CP16XX_IOCDMAW\n"); break;
        case CP16XX_IOC_GET_L2_DMA_PHYS_ADDR: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_GET_L2_DMA_PHYS_ADDR\n"); break;
        case CP16XX_IOC_OAPP:    DPRLIBLOGMSG("begin, cmd CP16XX_IOC_OAPP\n"); break;
        case CP16XX_IOC_CAPP:    DPRLIBLOGMSG("begin, cmd CP16XX_IOC_CAPP\n"); break;
        case CP16XX_IOC_BIND:    DPRLIBLOGMSG("begin, cmd CP16XX_IOC_BIND\n"); break;
        case CP16XX_IOC_UNBIND:  DPRLIBLOGMSG("begin, cmd CP16XX_IOC_UNBIND\n"); break;
        case CP16XX_IOC_IRTCBF:  DPRLIBLOGMSG("begin, cmd CP16XX_IOC_IRTCBF\n"); break;
        case CP16XX_IOC_SET_DMA_RANGE: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_SET_DMA_RANGE\n"); break;
        case CP16XX_IOC_GET_CP_TIMER:  DPRLIBLOGMSG("begin, cmd CP16XX_IOC_GET_CP_TIMER\n"); break;
        case CP16XX_IOCCOUNT:          DPRLIBLOGMSG("begin, cmd CP16XX_IOCCOUNT\n"); break;
        case CP16XX_IOC_PACKET_READ:   DPRLIBLOGMSG("begin, cmd CP16XX_IOC_PACKET_READ\n"); break;
        case CP16XX_IOC_PACKET_WRITE:  DPRLIBLOGMSG("begin, cmd CP16XX_IOC_PACKET_WRITE\n"); break;
        case CP16XX_IOC_OWD: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_OWD\n"); break;
        case CP16XX_IOC_CWD: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_CWD\n"); break;
        case CP16XX_IOC_TWD: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_TWD\n"); break;
        case CP16XX_IOC_GET_PNIO_DMA_RANGE: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_GET_PNIO_DMA_RANGE\n"); break;
        case CP16XX_IOC_IRTCPID: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_IRTCPID\n"); break;
        case CP16XX_IOC_MMAP:    DPRLIBLOGMSG("begin, cmd CP16XX_IOC_MMAP\n"); break;
        case CP16XX_IOC_MUNMAP: DPRLIBLOGMSG("begin, cmd CP16XX_IOC_MUNMAP\n"); break;
        default:                DPRLIBLOGMSG("begin, cmd 0x%x ?\n", cmd);
    }

    switch ( cmd ) {
        case CP16XX_IOC_OAPP: /* open(register) application */

            DPR_ASSERT(interface->card && !interface->app);

            if ( size_out != sizeof(struct t_register_app) ) {
                DPRLIBLOGMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_register_app));
                return E_CP16XX_WRONGARG;
            }
            if ( size_in != sizeof(struct t_register_app) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_register_app));
                return E_CP16XX_WRONGARG;
            }

            user_struct_app = (struct t_register_app *)arg;
            ret = sizeof(struct t_register_app);

            DPRLIBLOGMSG("CP16XX_IOC_OAPP flags %lu\n", user_struct_app->flags);

            switch ( user_struct_app->flags ) {
                case OAPP_APP_EXCLUSIVE:
                    open_app_type = APPLICATION_EXCLUSIVE;
                    break;
                case OAPP_APP_L2:
                    open_app_type = APPLICATION_L2;
                    break;
                default:
                    open_app_type = APPLICATION;
                    break;
            }

            interface->app = cp16xx_app_new(interface->card, open_app_type);
            if ( !interface->app ) /* to much applications */
                return E_CP16XX_MAXREACHED;

            user_struct_app->user_id = interface->app->user_id;
            break;

        case CP16XX_IOC_IRTCPID:
            DPR_ASSERT(interface->card && interface->app);

            if ( size_out != sizeof(struct t_register_app) ) {
                DPRLIBLOGMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_register_app));
                return E_CP16XX_WRONGARG;
            }

            user_struct_app = (struct t_register_app *)arg;
            ret = sizeof(struct t_register_app);

            user_struct_app->user_id = interface->card->cp_index;
            break;

        case CP16XX_IOC_CAPP: /* close(unregister) application */
            DPR_ASSERT(interface->card && interface->app);

            if ( size_in != sizeof(struct t_register_app) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_register_app));
                return E_CP16XX_WRONGARG;
            }

            user_struct_app = (struct t_register_app *)arg;

            if ( interface->app->user_id != user_struct_app->user_id )
                return E_CP16XX_WRONGUSERID;

            /*if(interface->app->wd_user == WD_USER_REF_HOSTAPP_MIN)
                cp16xx_wd_deactivate(interface->app);*/

            ret = cp16xx_app_free(interface->card, interface->app);
            break;

        case CP16XX_IOC_GET_CP_TIMER:  /* get cards timer */
            DPR_ASSERT(interface->card);

            if ( size_out != sizeof(struct t_read_timer) ) {
                DPRLIBLOGMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_read_timer));
                return E_CP16XX_WRONGARG;
            }

            tmp_timer = (struct t_read_timer *)arg;
            ret = sizeof(struct t_read_timer);
            tmp_timer->cp_timer = DPR_READ_UINT32(interface->card->bars[PCI_BAR_IRTE].bar_ptr + 0x11414);

            DPRLIBLOGMSG("cp_timer 0x%x\n", (unsigned int)tmp_timer->cp_timer);
            break;

        case CP16XX_IOC_GET_L2_DMA_PHYS_ADDR:  /* get dma physical address */
            DPR_ASSERT(interface->card);

            if ( !interface->card->l2.virt )
                return E_CP16XX_FAULT;

            if ( size_out != sizeof(struct t_dma_address) ) {
                DPRLIBLOGMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_dma_address));
                return E_CP16XX_WRONGARG;
            }

            user_struct_dma_address = (struct t_dma_address *)arg;
            ret = sizeof(struct t_dma_address);

            user_struct_dma_address->dma_address = DPR_PHYSICAL_ADDRESS_TO_PTR(interface->card->l2.phys);
            user_struct_dma_address->dma_size = interface->card->l2.size;
            break;

        case CP16XX_IOC_GET_PNIO_DMA_RANGE:  /* get dma physical address */
            DPR_ASSERT(interface->card);

            if ( size_out != sizeof(struct t_dma_address) ) {
                DPRLIBLOGMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_dma_address));
                return E_CP16XX_WRONGARG;
            }

            user_struct_dma_address = (struct t_dma_address *)arg;
            ret = sizeof(struct t_dma_address);

            user_struct_dma_address->dma_address = DPR_PHYSICAL_ADDRESS_TO_PTR(interface->card->irt.phys);
            user_struct_dma_address->dma_size = interface->card->irt.size;
            break;

        case CP16XX_IOC_IRTCBF:
            DPR_ASSERT(interface->app);

            if ( size_in != sizeof(struct t_register_app) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_register_app));
                return E_CP16XX_WRONGARG;
            }

            user_struct_app = (struct t_register_app *)arg;

            if ( interface->app->user_id != user_struct_app->user_id )
                return E_CP16XX_WRONGUSERID;

            DPR_SPINLOCK_LOCK(interface->card->apps_lock, flags);
            switch ( user_struct_app->flags ) {
                case DEL_STARTOP:
                    if ( interface->card->startop_app != interface->app ) {
                        ret = E_CP16XX_WRONGUSERID;
                    } else {
                        interface->card->startopfct = cp16xx_irt_startop_cbf_default;
                        interface->card->startop_app = NULL;
                    }
                    break;
                case REG_STARTOP:
                    if ( !interface->card->startop_app ) {
                        interface->card->startop_app = interface->app;
                    } else {
                        ret = E_CP16XX_MAXREACHED;
                    }
                    break;
                case KICKUP_STARTOP:
                    cp16xx_irt_startop_cbf(interface->card);
                    break;
                case RUN_STARTOP:
                    if ( interface->card->startop_app != interface->app ) {
                        ret = E_CP16XX_WRONGIOCTL;
                    } else {
                        interface->card->startopfct = cp16xx_irt_startop_cbf;
                    }
                    break;
                case DEL_OPFAULT:
                    if ( interface->card->opfault_app != interface->app ) {
                        ret = E_CP16XX_WRONGIOCTL;
                    } else {
                        interface->card->opfaultfct = cp16xx_irt_opfault_cbf_default;
                        interface->card->opfault_app = NULL;
                    }
                    break;
                case REG_OPFAULT:
                    if ( !interface->card->opfault_app ) {
                        interface->card->opfault_app = interface->app;
                    } else {
                        ret = E_CP16XX_MAXREACHED;
                    }
                    break;
                case KICKUP_OPFAULT:
                    cp16xx_irt_opfault_cbf(interface->card);
                    break;
                case RUN_OPFAULT:
                    if ( interface->card->opfault_app != interface->app ) {
                        ret = E_CP16XX_WRONGIOCTL;
                    } else {
                        interface->card->opfaultfct = cp16xx_irt_opfault_cbf;
                    }
                    break;
                case DEL_NEWCYCLE:
                    if ( interface->card->newcycle_app != interface->app ) {
                        ret = E_CP16XX_WRONGIOCTL;
                    } else {
                        interface->card->newcyclefct = cp16xx_irt_newcycle_cbf_default;
                        interface->card->newcycle_app = NULL;
                        cp16xx_irq_restore_mask_irt(interface->card);
                    }
                    break;
                case REG_NEWCYCLE:
                    if ( !interface->card->newcycle_app ) {
                        interface->card->newcycle_app = interface->app;
                        cp16xx_irq_restore_mask_irt(interface->card);
                    } else {
                        ret = E_CP16XX_MAXREACHED;
                    }
                    break;
                case KICKUP_NEWCYCLE:
                    cp16xx_irt_newcycle_cbf(interface->card);
                    break;
                case RUN_NEWCYCLE:
                    if ( interface->card->newcycle_app != interface->app ) {
                        ret = E_CP16XX_WRONGIOCTL;
                    } else {
                        interface->card->newcyclefct = cp16xx_irt_newcycle_cbf;
                    }
                    break;
                default:
                    ret = E_CP16XX_WRONGIOCTL;
            };
            DPR_SPINLOCK_UNLOCK(interface->card->apps_lock, flags);
            break;

        case CP16XX_IOC_SET_DMA_RANGE:
            DPR_ASSERT(interface->app);

            if ( size_in != sizeof(struct t_dma_range) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_dma_range));
                return E_CP16XX_WRONGARG;
            }

            user_dma_range = (struct t_dma_range *)arg;

            if ( interface->app->user_id != user_dma_range->user_id )
                return E_CP16XX_WRONGUSERID;

            DPR_SPINLOCK_LOCK(interface->card->apps_lock, flags);
            memcpy(&interface->app->dma_range, user_dma_range, sizeof(struct t_dma_range));
            DPR_SPINLOCK_UNLOCK(interface->card->apps_lock, flags);
            break;

        case CP16XX_IOCSHUTDOWN: /* reset card soft */
        case CP16XX_IOCRESET:    /* reset card hard */
            DPR_ASSERT(interface->card);

            if ( cmd == CP16XX_IOCSHUTDOWN ) {
                ret = DPR_ATOMIC_READ(interface->card->applications_count);
                DPRLIBLOGMSG("applications_count=%d\n", ret);

                /*
                 * L2 applications should not prevent the card from
                 * being resetted
                 */
                for ( i = 0; i < MAX_APPLICATIONS; ++i ) {
                    if ( interface->card->apps[i].user_id != FREE_ID &&
                         interface->card->apps[i].type == APPLICATION_L2 ) {
                        ret--;
                    }
                }

                DPRLIBLOGMSG("after escape l2app applications_count=%d, watchdog_cycle=%d\n", ret, watchdog_cycle);
                /* one application can be driver watchdog */
                if ( (watchdog_cycle && ret > 1) || (!watchdog_cycle && ret) ) {
                    DPRLIBERRMSG("CP16XX_IOCSHUTDOWN ERR: E_CP16XX_BUSY");
                    return E_CP16XX_BUSY;
                }
            } else {
                if ( !DPR_IS_ADMIN ) {
                    DPRLIBERRMSG("CP16XX_IOCRESET ERR: E_CP16XX_NOPERMISSION");
                    return E_CP16XX_NOPERMISSION;
                }
            }

#ifdef CP16XX_COMBINED_DRIVER
            if ( cp16xx_os_reset_begin(interface->card) != 0 )
                return E_CP16XX_BUSY;
#endif

            cp16xx_daemon_work(interface->card, DPRLIB_RESET_FW);
            ret = 0;
            break;

        case CP16XX_IOCREGR:
        case CP16XX_IOCREGW:
        case CP16XX_IOCDPRAMR:
        case CP16XX_IOCDPRAMW:
        case CP16XX_IOCDMAR:
        case CP16XX_IOCDMAW:
        case CP16XX_IOCCOUNT:

            if ( !DPR_IS_ADMIN )
                return E_CP16XX_NOPERMISSION;

            DPR_ASSERT(interface->card);

            switch ( cmd ) {
                case CP16XX_IOCREGR:
                case CP16XX_IOCREGW:
                case CP16XX_IOCDPRAMR:
                case CP16XX_IOCDPRAMW:

                    if ( size_in != sizeof(struct t_rw_direct) ) {
                        DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_rw_direct));
                        return E_CP16XX_WRONGARG;
                    }
                    user_struct_direct = (struct t_rw_direct *)arg;

                    switch ( cmd ) {
                        case CP16XX_IOCREGR:
                        case CP16XX_IOCREGW:
                            tmp = interface->card->bars[PCI_BAR_IRTE].bar_ptr + user_struct_direct->offset;
                            break;
                        case CP16XX_IOCDPRAMR:
                        case CP16XX_IOCDPRAMW:
                            tmp = interface->card->bars[PCI_BAR_DPRAM].bar_ptr + user_struct_direct->offset;
                            break;
                    }

                    /* we need to buffer write and read accesses */
                    buffer = DPR_VMALLOC(user_struct_direct->length);
                    if ( !buffer )
                        return E_CP16XX_NOMEMORY;

                    switch ( cmd ) {
                        case CP16XX_IOCREGW:
                        case CP16XX_IOCDPRAMW:
                            if ( DPR_MEMCPY_FROM_USER(buffer, user_struct_direct->data, user_struct_direct->length) ||
                                 DPR_MEMCPY_TO_PCI(tmp, buffer, user_struct_direct->length) )
                                ret = E_CP16XX_IOFAULT;
                            else
                                ret = 0;

                            break;
                        case CP16XX_IOCREGR:
                        case CP16XX_IOCDPRAMR:
                            if ( DPR_MEMCPY_FROM_PCI(buffer, tmp, user_struct_direct->length) ||
                                 DPR_MEMCPY_TO_USER(user_struct_direct->data, buffer, user_struct_direct->length) )
                                ret = E_CP16XX_IOFAULT;
                            else
                                ret = sizeof(struct t_rw_direct);
                            break;
                    };

                    DPR_VFREE(buffer);
                    break;

                case CP16XX_IOCDMAR:
                case CP16XX_IOCDMAW:

                    if ( size_in != sizeof(struct t_rw_direct) ) {
                        DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_rw_direct));
                        return E_CP16XX_WRONGARG;
                    }
                    user_struct_direct = (struct t_rw_direct *)arg;

                    switch ( cmd ) {
                        case CP16XX_IOCDMAR:
                            if ( DPR_MEMCPY_TO_USER(user_struct_direct->data,
                    (char *)interface->card->irt.virt + user_struct_direct->offset, user_struct_direct->length))
                                ret = E_CP16XX_IOFAULT;
                            else
                                ret = sizeof(struct t_rw_direct);
                            break;
                        case CP16XX_IOCDMAW:
                if(DPR_MEMCPY_FROM_USER((char *)interface->card->irt.virt + user_struct_direct->offset,
                                                      user_struct_direct->data, user_struct_direct->length) )
                                ret = E_CP16XX_IOFAULT;
                            else
                                ret = 0;
                            break;
                    }
                    break;

                case CP16XX_IOCCOUNT:

                    if ( size_out != sizeof(struct t_read_pool) ) {
                        DPRLIBLOGMSG("wrong size_out argument = %lu, must be %lu\n", size_out, (unsigned long)sizeof(struct t_read_pool));
                        return E_CP16XX_WRONGARG;
                    }

                    user_pool = (struct t_read_pool *)arg;
                    ret = sizeof(struct t_read_pool);
                    user_pool->user_id = interface->card->cp.loop_counter;
                    break;

                default:
                    DPR_ASSERT(!"check case nesting");
            }
            break;

#ifdef SWITCH_DIAG_TRACE

        case CP16XX_IOC_SWITCH_DIAG: /* get switch diag. trace */
            DPR_ASSERT(interface->card);

            if ( size_in != sizeof(struct t_rw_direct) ) {
                DPRLIBLOGMSG("wrong size_in argument = %lu, must be %lu\n", size_in, (unsigned long)sizeof(struct t_rw_direct));
                return E_CP16XX_WRONGARG;
            }
            user_switch_diag_trc = (struct t_switch_diag_trc *)arg;

            if ( cp16xx_get_switch_diag_trc(interface->card, user_switch_diag_trc) ) {
                ret = E_CP16XX_FAULT;
                break;
            }

            /* memset(card->trc.virt, 0x33, user_switch_diag_trc.buf_len); */

            if ( DPR_MEMCPY_TO_USER(
                                   DPR_ULONG_TO_PTR(user_switch_diag_trc->buf_offset),
                                   DPR_ULONG_TO_PTR(((DPR_PTR_TO_ULONG(interface->card->trc.virt) + 8 /*TCW 1,2*/)+7)&~7),
                                   user_switch_diag_trc->buf_len) )
                ret = E_CP16XX_IOFAULT;
            else
                ret = user_switch_diag_trc->buf_len;

            break;

#endif /* SWITCH_DIAG_TRACE */

        default:
            ret = E_CP16XX_WRONGIOCTL;
    }

    if ( ret < 0 )
        DPRLIBERRMSG("cmd=0x%x ret=%d\n", cmd, ret);
    else
        DPRLIBLOGMSG("end ret=%d\n", ret);

    return ret;
}

int cp16xx_base_ioctl(struct cp16xx_interface *interface,
                      unsigned int cmd, void *arg, unsigned long size_in, unsigned long size_out)
{
    if ( interface->channel_number != CONTROL_INTERFACE )
        return cp16xx_base_ioctl_1(interface, cmd, arg, size_in, size_out);
    else
        return cp16xx_base_ioctl_2(interface, cmd, arg, size_in, size_out);
}

int cp16xx_base_read(struct cp16xx_interface *interface, const char *buf, int count)
{
    int ret = 0;

    DPRLIBCHATMSG("begin\n");

    DPR_ASSERT(interface && interface->card );

    DPRLIBCHATMSG("%p (channel %d) card %p app %p buf %p count %d \n",
                 interface, interface->channel_number, interface->card, interface->app, buf, count);


    if ( !IS_VALID_USER_ID(interface->app) ) {
        DPRLIBLOGMSG("app invalid USER_ID\n");
        return E_CP16XX_WRONGUSERID;
    }

    if ( interface->channel_number == CONTROL_INTERFACE ) {
        switch ( count ) {
            case CP16XX_STARTOP:
                if ( interface->card->startop_app != interface->app )
                    return E_CP16XX_NOPERMISSION;
                break;
            case CP16XX_OPFAULT:
                if ( interface->card->opfault_app != interface->app )
                    return E_CP16XX_NOPERMISSION;
                break;
            case CP16XX_NEWCYCLE:
                if ( interface->card->newcycle_app != interface->app )
                    return E_CP16XX_NOPERMISSION;
                break;
            default:
                return E_CP16XX_WRONGARG;
        }
    } else {
        DPR_ASSERT(interface->channel);

        if ( !IS_VALID_USER_ID(interface->channel) ) {
            DPRLIBLOGMSG("channel invalid USER_ID, just closed?\n");
            return E_CP16XX_WRONGUSERID;
        }
        if ( interface->app->user_id != interface->channel->user_id ) {
            DPRLIBERRMSG("different USER_ID: app reference %lu, channel reference %lu\n",
                         interface->app->user_id, interface->channel->user_id);
            return E_CP16XX_WRONGUSERID;
        }
    }

    DPRLIBCHATMSG("end\n");

    return ret;
}

int cp16xx_base_write(struct cp16xx_interface *interface, const char *buf, int count)
{
    int ret = 0;

    DPRLIBCHATMSG("begin\n");

    DPR_ASSERT(interface && interface->card && interface->app);

    if ( !IS_VALID_USER_ID(interface->app) ) {
        DPRLIBLOGMSG("app invalid USER_ID\n");
        return E_CP16XX_WRONGUSERID;
    }

    if ( interface->channel_number == CONTROL_INTERFACE ) {
        if ( count == CP16XX_OPDONE ) {
            cp16xx_irt_set_opdone(interface->card);
            ret = count;
        }
    } else {
        if ( !IS_VALID_USER_ID(interface->channel) ) {
            DPRLIBLOGMSG("channel invalid USER_ID, just closed?\n");
            return E_CP16XX_WRONGUSERID;
        }
        if ( interface->app->user_id != interface->channel->user_id ) {
            DPRLIBERRMSG("different USER_ID: app reference %lu, channel reference %lu\n",
                         interface->app->user_id, interface->channel->user_id);
            return E_CP16XX_WRONGUSERID;
        }

        ret = DPRLIB_channel_write_message(&interface->card->cp,
                                           interface->channel_number, interface->app->user_id, (void *)buf, count);

        ret = (ret == DPR_ERROR) ? 0 : count;
    }

    DPRLIBCHATMSG("end\n");

    return ret;
}

/* cp16xx_base_read_cbf - will be called from soft interrupt context to
   handle incoming packets from FW. Handling is very different in some channels */
int cp16xx_base_read_cbf(void *arg, DPR_CHN_TYPE channel_number,
                         DPR_UINT32 user_id, unsigned long msg_length)
{
    CpData *pCP = (CpData *)arg;
    struct cp16xx_card_data *card;
    struct cp16xx_app_data *app;
    struct cp16xx_channel *channel;
    struct cp16xx_block *block;
    int handled = 0;

    DPRLIBCHATMSG("begin: ch-num=%d usr-id=%d len=%lu \n", channel_number, user_id, msg_length);

    card = (struct cp16xx_card_data *)pCP->parent;
    if ( !card ) {
        DPRLIBERRMSG("uninitialized CP reference\n");
        return DPR_ERROR;
    }

    block = BLOCK_create(msg_length);
    if ( !block ) {
        DPRLIBERRMSG("unable to alloc block %lu bytes\n", msg_length);
        return DPR_ERROR;
    }
// for test only
//   DPR_TASK_DELAY_UNINTERRUPTIBLE(3000);    // wait un-interruptible millisecs

    if ( DPR_OK != DPRLIB_channel_read_message(pCP, channel_number,
                                               block->addr, block->length) ) {
        BLOCK_free(block);
        DPRLIBERRMSG("DPRLIB_channel_read_message(channel %d, %lu bytes)\n",
                     channel_number, block->length);
        return DPR_ERROR;
    }

    if ( DPR_MGT_CHN == channel_number ) {
        DPR_MSG_HDR *rqb = (DPR_MSG_HDR *)block->addr;
        user_id = LE_TO_CPU(rqb->hostref);
        if ( !user_id ) {
            DPRLIBERRMSG("DPR_MGT_CHN user id NULL\n");
            DPRLIBERRMSG("hostdriver_ref %u\n", LE_TO_CPU(rqb->hostref));
            DPRLIBERRMSG("subsystem 0x%02x\n", rqb->subsystem);
            DPRLIBERRMSG("req_id 0x%08x\n", LE_TO_CPU(rqb->userid));
            DPRLIBERRMSG("response 0x%x\n", LE_TO_CPU(rqb->response));
            DPRLIBERRMSG("length %u\n", LE_TO_CPU(rqb->userdatalength));
        }
    } else if ( DPR_PNIO_CMD_CHN == channel_number ) {
#ifdef DEBUG
        static DPR_UINT32 orderid = 0;
        static DPR_UINT32 read_offset = 0;
        DPR_UINT32 opcode = ((DPR_UINT32 *)(block->addr))[1];
        DPR_UINT32 current_orderid = ((DPR_UINT32 *)(block->addr))[2];
        DPR_UINT32 current_read_offset = DPR_READ_UINT32(&card->cp.pDpramConfigArea->Rings[1].pRead);

        if ( opcode == 1 ) {
            orderid = 0;
        } else if ( opcode > 1 && opcode < 100 ) {
            if ( current_orderid != orderid+1 ) {
                unsigned int i;
                char *p;
                DPRLIBERRMSG("DPR_PNIO_CMD_CHN, opcode %u, wrong sequence, last packet was 0x%08x, current 0x%08x, last read 0x%08x, current_read 0x%08x\n",
                             opcode, orderid, current_orderid, read_offset, current_read_offset);
                for ( i = 0, p = block->addr; i < block->length; i++, p++ ) {
                    if ( !(i % 16) )
                        DPRLIBERRMSG("\ndata 0x%lx: ", (unsigned long)(p - block->addr));
                    DPRLIBERRMSG(" %02X", *p);
                }
                DPRLIBERRMSG("\n");
            }
            orderid = current_orderid;
        }
        read_offset = current_read_offset;
#endif  /* DEBUG */
    }

    DPRLIBCHATMSG("search for application_id %u, channel %d\n",
                 user_id, channel_number);
    app = cp16xx_app_search(card, user_id);
    DPRLIBCHATMSG("found app %p\n", app);

    if ( !app ) {
        DPRLIBLOGMSG("invalid application_id %u, propably already closed\n", user_id);
        BLOCK_free(block);
        goto cp16xx_cbf_read_and_fail;
    }

    DPR_ASSERT(app->parent == card);
    channel = &app->pools[channel_number];
    if ( DPR_MGT_CHN != channel_number ) {
        DPR_ASSERT(app->user_id == channel->user_id);
        DPR_ASSERT(channel->parent == app);
        DPR_ASSERT(channel->channel_number == channel_number);
    }

    if ( DPR_MGT_CHN == channel_number ) {
        union {
            char *tmp;
            DPR_MSG_HDR *rqb;
            WD_INIT_DPRMGTCHNL_RB *wd_rqb;
            DPRMGTCHNL_TO_MGT_RB *mgt_rqb;
        } rq;

        rq.tmp = block->addr;

#ifdef DEBUG
        DPRLIBLOGMSG("hostdriver_ref %u\n", LE_TO_CPU(rq.rqb->hostref));
        DPRLIBLOGMSG("subsystem 0x%02x\n", rq.rqb->subsystem);
        DPRLIBLOGMSG("req_id 0x%08x\n", LE_TO_CPU(rq.rqb->userid));
        DPRLIBLOGMSG("response 0x%x\n", LE_TO_CPU(rq.rqb->response));
        DPRLIBLOGMSG("length %u\n", LE_TO_CPU(rq.rqb->userdatalength));
#endif  /* DEBUG */

        switch ( rq.rqb->subsystem ) {
            case FW_SUBSYS_WATCHDOG:
                DPRLIBLOGMSG("wd_opcode %u\n", LE_TO_CPU16(rq.wd_rqb->wd_opcode));
                switch ( LE_TO_CPU16(rq.wd_rqb->wd_opcode) ) {
                    case WD_OPC_INIT_CNF:
                    case WD_OPC_SHUT_CNF:
                        if ( sizeof(*rq.wd_rqb) != block->length )
                            DPRLIBERRMSG("get more data %lu bytes as expected %lu bytes\n",
                                         block->length, (unsigned long)sizeof(*rq.wd_rqb));
                        if ( app->mgt_synch_data ) {
                            memcpy(app->mgt_synch_data, block->addr, block->length);
                            app->mgt_synch_data = NULL;
                            DPRLIBLOGMSG("wd_ref 0x%x\n", LE_TO_CPU(rq.wd_rqb->wd_ref));
                            DPRLIBLOGMSG("wd_user_ref %u\n", LE_TO_CPU(rq.wd_rqb->wd_user_ref));
                            DPR_SEM_POST(app->mgt_synch);
                        } else {
                            DPRLIBERRMSG("unable to save wd synch block\n");
                        }
                        handled = 1;

                        break;

                    default:
                        break;
                }
                break;

            case FW_SUBSYS_CP_MGT:
                DPRLIBLOGMSG("mgt_opcode %u\n", LE_TO_CPU16(rq.mgt_rqb->mgt_opcode));
                switch ( LE_TO_CPU16(rq.mgt_rqb->mgt_opcode) ) {
                    case OPC_MGT_HOST_ABORT_CNF:
                        DPR_ASSERT(sizeof(DPRMGTCHNL_TO_MGT_RB) == block->length);
                        if ( app->mgt_synch_data ) {
                            memcpy(app->mgt_synch_data, block->addr, block->length);
                            app->mgt_synch_data = NULL;
                            DPR_SEM_POST(app->mgt_synch);
                        } else {
                            DPRLIBERRMSG("unable to save mgt synch block\n");
                        }
                        handled = 1;
                        break;

                    case OPC_MGT_HOST_SET_DMA_CNF:
                        DPR_ASSERT(sizeof(DPRMGTCHNL_TO_MGT_SET_DMA) == block->length);
                        if ( app->mgt_synch_data ) {
                            memcpy(app->mgt_synch_data, block->addr, block->length);
                            app->mgt_synch_data = NULL;
                            DPR_SEM_POST(app->mgt_synch);
                        } else {
                            DPRLIBERRMSG("unable to save mgt synch block\n");
                        }
                        handled = 1;
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }

    if ( handled ) {
        BLOCK_free(block);
    } else {
        if ( !BLOCK_add_tail(channel, block) ) {
            BLOCK_free(block);
            DPRLIBERRMSG("unable to queue block 0x%p, channel %d\n", block, channel_number);
        } else {
            cp16xx_os_read_complete(channel);
        }
    }

    cp16xx_cbf_read_and_fail:

    DPRLIBCHATMSG("end\n");

    return DPR_OK;
}

const char *cp16xx_strnstr(const char *_Str, const char *_SubStr, size_t StrLen)
{
    size_t i;
    for ( i=0; i<StrLen-strlen(_SubStr); i++ ) {
        if ( memcmp(_SubStr, _Str + i, strlen(_SubStr)) == 0 ) {
            return _Str+i;
        }
    }

    return 0;
}

static int cp16xx_check_fw_info(struct cp16xx_app_data *app)
{
#define EXCEPTION_BUFFER_START  0x504100UL
#define EXCEPTION_PREAMBLE   "EXCEPTION:\n"

    struct cp16xx_card_data *card = (struct cp16xx_card_data *)app->parent;
    char *fw_info = card->bars[PCI_BAR_DPRAM].bar_ptr + EXCEPTION_BUFFER_START;
    const char *pfile, *pfile_end, *pline_nr;
    long line_number = 0;
    char file_name[128];
    char *tmp = app->wd_buf;

    memset(file_name, 0, sizeof(file_name));

    DPR_MEMCPY_FROM_PCI(tmp, fw_info, strlen(EXCEPTION_PREAMBLE));
    if ( memcmp(tmp, EXCEPTION_PREAMBLE, strlen(EXCEPTION_PREAMBLE)) == 0 ) {
        DPR_MEMCPY_FROM_PCI(tmp, fw_info, 1024);
        pfile = cp16xx_strnstr(tmp, "file ", 512);
        if ( pfile ) {

            pfile += strlen("file ");

            strncpy(file_name, pfile, 20);

            pfile_end = cp16xx_strnstr(pfile, ",", 128);
            if ( pfile_end ) {
                unsigned long file_name_len = DPR_PTR_TO_ULONG(pfile_end) - DPR_PTR_TO_ULONG(pfile);
                if ( file_name_len > (sizeof(file_name) - 1) )
                    file_name_len = sizeof(file_name) - 1;
                strncpy(file_name, pfile, file_name_len);
            } else {
                DPRLIBERRMSG("file name in EXEPTION INFO found, but no ,\n");
            }

            pline_nr = cp16xx_strnstr(pfile_end, "line ", 16);
            if ( pline_nr ) {
                line_number = atol(pline_nr + strlen("line "));
            } else {
                DPRLIBERRMSG("line number in EXEPTION INFO not found\n");
            }
        } else {
            DPRLIBERRMSG("file name in EXEPTION INFO not found\n");
        }

        DPRLIBERRMSG("Important firmware message %lu, %s\n", line_number, file_name);
        cp16xx_syslog(file_name, line_number, "Firmware message");
        return 1;
    }

    return 0;
}

/* helper functions to provide watchdog support

   cp16xx_wd_timer - driver callback serves <driver observes FW> and <FW observes driver>
                     watchdogs.

   cp16xx_wd_trigger - checks and/or informs about watchdogs misses

   cp16xx_wd_activate - register watchdog by FW(mean all sorts of watchogs, application level too)
   cp16xx_wd_deactivate - release watchdog
   cp16xx_wd_deactivate_all_wds - helper to free all watchdogs at once

   cp16xx_wd_observe - helper to wrap HW acesses. Used in cp16xx_wd_timer() too */
static void cp16xx_wd_timer(void *arg)
{
    struct cp16xx_app_data *app = (struct cp16xx_app_data *)arg;
    int ret;

    if ( WD_FLAG_HOST == app->wd_owner ) {
        ret = cp16xx_wd_observe(app);
        if ( ret ) {
            struct cp16xx_card_data *card = (struct cp16xx_card_data *)app->parent;
            if ( !card->firmware_timeouted )
                DPRLIBERRMSG("Firmware timeout\n");
            card->firmware_timeouted = 1;
        }
    } else if ( WD_FLAG_FIRMWARE == app->wd_owner ) {
        ret = cp16xx_wd_trigger(app);
    } else {
        ret = -1;
        DPRLIBERRMSG("unknown watchdog\n");
    }

    if ( !app->important_fw_msg_reported && cp16xx_check_fw_info(app) ) {
        app->important_fw_msg_reported = 1;
    }

    if ( !ret && app->wd_cell && app->wd_kernel_timer ) {
        DPR_TIMER_START(app->wd_kernel_timer, app->wd_timeout);
    }
}

static int cp16xx_wd_trigger(struct cp16xx_app_data *app)
{
    if ( app->wd_cell ) {
        if ( WD_FLAG_RUNNING == DPR_READ_UINT8(&app->wd_cell->wd_flag) ) {
            DPR_WRITE_UINT8(WD_TRIGGER_SET, &app->wd_cell->wd_trigger);
            return 0;
        } else {
            DPRLIBINFMSG("timeout detected, watchdog cell at 0x%p, wd_ref %u\n",
                         app->wd_cell, app->wd_ref);
        }
    }

    return -1;
}

static int cp16xx_wd_activate(struct cp16xx_app_data *app)
{
    static const WD_SUBSYSTEM agents[2] =
    {
        { FW_SUBSYS_PNIO_CTRL, 0},
        { FW_SUBSYS_PNIO_DEVICE, 0}
    };
    int ret = 0;
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)app->parent;
    unsigned short flags;
    WD_INIT_DPRMGTCHNL_RB rqb;

    DPRLIBLOGMSG("begin\n");

    if ( !IS_VALID_USER_ID(app) )
        return -1;

    if ( app->wd_cell ) {
        DPRLIBERRMSG("already initialized: no action - exit\n");
        return -1;
    }

    if ( WD_USER_REF_HOSTDRIVER == app->wd_user ) {
        if ( !(app->wd_kernel_timer = (DPR_TIMER *)DPR_ZALLOC(sizeof(DPR_TIMER))) ) {
            DPRLIBERRMSG("failed initialize in driver timer\n");
            return -2;
        }
    }

    rqb.header.hostref = CPU_TO_LE(app->user_id);
    rqb.header.subsystem = FW_SUBSYS_WATCHDOG;
    rqb.header.userid = 0xaabbccdd;
    rqb.header.response = 0;
    rqb.header.userdatalength = CPU_TO_LE(CalculateLengthRbInit(2));

    rqb.wd_opcode = CPU_TO_LE16(WD_OPC_INIT_REQ);

    flags = app->wd_owner | WD_FLAG_NOTIFY_SUBSYSTEM;
    if ( WD_USER_REF_HOSTDRIVER != app->wd_user )
        flags |= WD_FLAG_NOTIFY_HOST;
    rqb.wd_flags = CPU_TO_LE16(flags);
    rqb.wd_user_ref = CPU_TO_LE(app->wd_user);
    rqb.wd_timeout = CPU_TO_LE(app->wd_timeout);
    rqb.wd_ref = CPU_TO_LE(WD_REF_INVALID);
    rqb.wd_ss_count = CPU_TO_LE16(2);
    rqb.wd_ss[0] = agents[0];
    rqb.wd_ss[1] = agents[1];

    DPRLIBLOGMSG("hostdriver_ref %u\n", LE_TO_CPU(rqb.header.hostref));
    DPRLIBLOGMSG("subsystem 0x%02x\n", rqb.header.subsystem);
    DPRLIBLOGMSG("req_id 0x%08x\n", LE_TO_CPU(rqb.header.userid));
    DPRLIBLOGMSG("response 0x%x\n", LE_TO_CPU(rqb.header.response));
    DPRLIBLOGMSG("length %u\n", LE_TO_CPU(rqb.header.userdatalength));
    DPRLIBLOGMSG("wd_opcode %u\n", LE_TO_CPU16(rqb.wd_opcode));
    DPRLIBLOGMSG("wd_flags 0x%x\n", LE_TO_CPU16(rqb.wd_flags));
    DPRLIBLOGMSG("wd_user_ref %u\n", LE_TO_CPU(rqb.wd_user_ref));
    DPRLIBLOGMSG("wd_timeout %u\n", LE_TO_CPU(rqb.wd_timeout));
    DPRLIBLOGMSG("wd_ref 0x%x\n", LE_TO_CPU(rqb.wd_ref));
    DPRLIBLOGMSG("wd_ss_count %u\n", LE_TO_CPU16(rqb.wd_ss_count));

    app->mgt_synch_data = &rqb;
    if ( DPR_OK != DPRLIB_channel_write_message(&card->cp,
                                                DPR_MGT_CHN, 0, &rqb, sizeof(rqb)) ) {
        app->mgt_synch_data = NULL;
        DPRLIBERRMSG("fail to send over mgt channel\n");
        ret = -1;
        goto cp16xx_wd_activate_exit;
    }

    ret = DPR_SEM_WAIT_FOR_WAKEUP(app->mgt_synch, 1000); // wait 1 sec. for response
    if ( ret != 0 ) {
        app->mgt_synch_data = NULL;
        DPRLIBERRMSG("interrupted or timeout, ret=%d (-1=interrupt, 1=timeout) \n", ret);
        ret = -1;
        goto cp16xx_wd_activate_exit;
    }

    if ( LE_TO_CPU16(rqb.wd_opcode) != WD_OPC_INIT_CNF) {
        DPRLIBERRMSG("invalid opcode=%d expected=%d\n",LE_TO_CPU16(rqb.wd_opcode), WD_OPC_INIT_CNF);
        ret = -1;
        goto cp16xx_wd_activate_exit;
    }

    if ( rqb.header.response ) {
        DPRLIBERRMSG("got an invalid response 0x%x\n", LE_TO_CPU(rqb.header.response));
        ret = -1;
        goto cp16xx_wd_activate_exit;
    }

    app->wd_ref = LE_TO_CPU(rqb.wd_ref);
    app->wd_cell = (DPRAM_WD *)(card->bars[PCI_BAR_DPRAM].bar_ptr +
                                0x001FFFBC + sizeof(DPRAM_WD) * app->wd_ref);

    if ( WD_USER_REF_HOSTDRIVER == app->wd_user ) {
        if ( WD_FLAG_FIRMWARE == app->wd_owner )
            app->wd_timeout = (app->wd_timeout / 2) + 1;

        DPR_TIMER_INIT(app->wd_kernel_timer, cp16xx_wd_timer, app);
        DPR_TIMER_START(app->wd_kernel_timer, app->wd_timeout);

        DPRLIBLOGMSG("wd ok started\n");
    }

    cp16xx_wd_activate_exit:
    DPRLIBLOGMSG("end: return=%d (0=ok)\n",ret);
    return ret;
} /* end of cp16xx_wd_activate */

static int cp16xx_wd_deactivate(struct cp16xx_app_data *app)
{
    int ret = 0;
    WD_SHUT_DPRMGTCHNL_RB rqb;
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)app->parent;

    DPRLIBLOGMSG("begin\n");

    if ( !app->wd_cell )
        goto cp16xx_wd_deactivate_exit;

    DPRLIBLOGMSG("app %p\n", app);
    DPRLIBLOGMSG("at %p, mgt_synch_data %p\n", &app->mgt_synch_data, app->mgt_synch_data);
    DPRLIBLOGMSG("at %p, wd_user %u\n", &app->wd_user, app->wd_user);
    DPRLIBLOGMSG("at %p, wd_ref %u\n", &app->wd_ref, app->wd_ref);
    DPRLIBLOGMSG("at %p, wd_timeout %u\n", &app->wd_timeout, app->wd_timeout);
    DPRLIBLOGMSG("at %p, wd_cell %p\n", &app->wd_cell, app->wd_cell);
    DPRLIBLOGMSG("at %p, wd_kernel_timer %p\n", &app->wd_kernel_timer, app->wd_kernel_timer);
    DPRLIBLOGMSG("at %p, wd_owner %u\n", &app->wd_owner, app->wd_owner);

    if ( app->wd_cell < (DPRAM_WD *)0x001FFFBC ) {
        DPRLIBLOGMSG("app %p, wd_cell %p\n", app, app->wd_cell);
        goto cp16xx_wd_deactivate_exit;
    }

    if ( WD_FLAG_RUNNING == DPR_READ_UINT8(&app->wd_cell->wd_flag) )
        DPR_WRITE_UINT8(WD_TRIGGER_SHUT, &app->wd_cell->wd_trigger);
    app->wd_cell = NULL;

    if ( WD_USER_REF_HOSTDRIVER == app->wd_user ) {
        /* stop and delete timer */
        if ( app->wd_kernel_timer ) {
            DPR_TIMER_DELETE_SYNC(app->wd_kernel_timer);
            DPR_FREE(app->wd_kernel_timer);
            app->wd_kernel_timer = NULL;
        }
    }

    if ( card->firmware_timeouted )
        goto cp16xx_wd_deactivate_freeapp;

    rqb.header.hostref = CPU_TO_LE(app->user_id);
    rqb.header.subsystem = FW_SUBSYS_WATCHDOG;
    rqb.header.userid = 0x11223344;
    rqb.header.response = 0;
    rqb.header.userdatalength = CPU_TO_LE(CalculateLengthRbShut(2));
    rqb.wd_opcode = CPU_TO_LE16(WD_OPC_SHUT_REQ);
    rqb.wd_flags = 0;
    rqb.wd_user_ref = CPU_TO_LE(app->wd_user);
    rqb.wd_timeout = 0;
    rqb.wd_ref = CPU_TO_LE(app->wd_ref);
    rqb.wd_ss_count = 0;

    DPRLIBLOGMSG("hostdriver_ref %u\n", LE_TO_CPU(rqb.header.hostref));
    DPRLIBLOGMSG("subsystem 0x%02x\n", rqb.header.subsystem);
    DPRLIBLOGMSG("req_id 0x%08x\n", LE_TO_CPU(rqb.header.userid));
    DPRLIBLOGMSG("response 0x%x\n", LE_TO_CPU(rqb.header.response));
    DPRLIBLOGMSG("length %u\n", LE_TO_CPU(rqb.header.userdatalength));
    DPRLIBLOGMSG("wd_opcode %u\n", LE_TO_CPU16(rqb.wd_opcode));
    DPRLIBLOGMSG("wd_flags 0x%x\n", LE_TO_CPU16(rqb.wd_flags));
    DPRLIBLOGMSG("wd_user_ref %u\n", LE_TO_CPU(rqb.wd_user_ref));
    DPRLIBLOGMSG("wd_timeout %u\n", LE_TO_CPU(rqb.wd_timeout));
    DPRLIBLOGMSG("wd_ref 0x%x\n", LE_TO_CPU(rqb.wd_ref));
    DPRLIBLOGMSG("wd_ss_count %u\n", LE_TO_CPU16(rqb.wd_ss_count));

    app->mgt_synch_data = &rqb;
    if ( DPR_OK != DPRLIB_channel_write_message(&card->cp,
                                                DPR_MGT_CHN, 0, &rqb, sizeof(rqb)) ) {
        DPRLIBERRMSG("fail to send over mgt channel\n");
        app->mgt_synch_data = NULL;
        ret = -1;
        goto cp16xx_wd_deactivate_freeapp;
    }

    if ( DPR_SEM_WAIT_FOR_WAKEUP(app->mgt_synch, 1000) ) {
        app->mgt_synch_data = NULL;
        DPRLIBLOGMSG("got a signal or timeout\n");
        ret = -1;
        goto cp16xx_wd_deactivate_freeapp;
    }

    DPR_ASSERT(!app->mgt_synch_data || &rqb == app->mgt_synch_data);
    DPR_ASSERT(LE_TO_CPU16(rqb.wd_opcode) == WD_OPC_SHUT_CNF);

    cp16xx_wd_deactivate_freeapp:
    cp16xx_app_free(card, app);

    cp16xx_wd_deactivate_exit:
    DPRLIBLOGMSG("end\n");

    return ret;
}

static int cp16xx_wd_deactivate_all_wds(struct cp16xx_card_data *card)
{
    struct cp16xx_app_data *app;
    int i;

    for ( i = 0; i < MAX_APPLICATIONS; i++ ) {
        app = &card->apps[i];
        if ( app->wd_cell )
            cp16xx_wd_deactivate(app);
    }
    return 0;
}

static int cp16xx_wd_observe(struct cp16xx_app_data *app)
{
    if ( app->wd_cell ) {
        if ( WD_TRIGGER_SET == DPR_READ_UINT8(&app->wd_cell->wd_trigger) )
            DPR_WRITE_UINT8(WD_TRIGGER_RESET, &app->wd_cell->wd_trigger);
        else if ( WD_TRIGGER_SHUT != DPR_READ_UINT8(&app->wd_cell->wd_trigger) ) {
            DPR_WRITE_UINT8(WD_FLAG_TIMEOUT, &app->wd_cell->wd_flag);
            return -1;
        }
    }

    return 0;
}

/* helper functions to provide card manipulation support from
   independent context(not application and not IRQ context)

   cp16xx_daemon_work - sets next work for daemon, and wakes him up

   cp16xx_daemon_timer - timer callback, provides sheduled repeating of
                         last daemon work without recursion

   cp16xx_daemon - daemon itself, provides 3 different works:
                   connect to FW
                   disconnect and quit
                   reboot of FW */
void cp16xx_daemon_work(void *arg, enum work work)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;

    DPRLIBLOGMSG(" next work=%d (0=START, 1=STOP, 2=REST_FW) \n",work);
/*
#ifdef CP16XX_COMBINED_DRIVER
  if (work == DPRLIB_RESET_FW) {
    if (cp16xx_os_reset(card) == -1)
      return;
  }
#endif
*/
    DPR_ATOMIC_SET(card->daemon_work, work);
    DPR_SEM_POST(card->daemon_resume);
}

void cp16xx_daemon_timer(void *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    DPR_SEM_POST(card->daemon_resume);
}


/*-----------------------------------------------------------------------------
 * Name  : cp16xx_daemon
 * Descr : Kernel task (thread) to handle 'jobs' in independent context.
 *         I.e. not in application and not in IRQ context.
 *         Job is set by the daemon thread itself (e.g. to reach a next state) or
 *         'job' is set by calling of the function cp16xx_daemon_work() above.
 *         Supported 'jobs' are defined in dprintern.h: enum work {...}
 *         DPRLIB_START:    - connect driver/lib to the firmware
 *         DPRLIB_STOP:     - disconnect driver/lib from the firmware.
 *         DPRLIB_RESET_FW: - disconnect from the firmware, see DPRLIB_STOP
 *                          - reset the firmware (card = hw reset)
 *                          - reconnect driver/lib to the firmware, see DPRLIB_START
 * Param :
 *  [ IN]: cp16xx_card_data* arg: Pointer to the hw (board) specific param struct.
 * Return: Always returns 0 == ok. Note: is a task function, returns only if killed.
 */
int cp16xx_daemon(void * arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)arg;
    enum work work;
    DPR_TIMER connect_timer;
    char tmp[256];
    int  fw_start_counter = 0;
    int  dprlib_start_counter = 0;

    DPR_THREAD_DAEMONIZE("cp16xx_daemon");
    DPR_THREAD_ALLOW_SIGNALS;
    DPR_TIMER_INIT(&connect_timer, cp16xx_daemon_timer, arg);
    DPRLIBLOGMSG("begin\n");

    while ( 1 )     {
        int ret = DPR_SEM_WAIT_INTERRUPTIBLE(card->daemon_resume);
        work = DPR_ATOMIC_READ(card->daemon_work);

        if ( ret ) {
           /* returns != 0: (-EINTR) wait interrupted, shutdown signal.
            * -> disconnect from the fw and exit the thread. work = DPRLIB_STOP
            */
            DPRLIBLOGMSG("daemon interrupted, closing fw connection. work=%d ret=%d\n", work, ret);
            work = DPRLIB_STOP;
        } else {
           /* returns zero: obtained the semaphore, do the work
            */
            DPRLIBLOGMSG("daemon triggered. work=%d (0=START 1=STOP 2=REST_FW)\n", work);

           /* dry semaphore out, let it work like a pulse (binary, not counted)
            * only the last posted job is performed, there is no job queue.
            * clean the semaphore, count it down
            * DPR_SEM_WAIT_TIME()  returns: -1, 0, 1
            *  -1: wait interrupted
            *   0: sema obtained (not empty, continue, try wait again)
            *   1: timeout - sema is empty, perform the work.
            */
            do {
                int i = DPR_SEM_WAIT_TIME(card->daemon_resume, 0);
                if ( i == -1 )
                    work = DPRLIB_STOP; /* interrupted */

                if ( i ) /* timeout, semaphore internal counter reaches zero */
                    break;

                /* i == 0: continue */
            } while ( 1 );
        }

        if ( DPRLIB_STOP == work || DPRLIB_RESET_FW == work ) {

            cp16xx_irq_restore_mask_rt(card);
            cp16xx_irq_restore_mask_irt(card);
            cp16xx_wd_deactivate_all_wds(card);

#ifdef CP16XX_COMBINED_DRIVER
            card->card_wd_app = NULL;
#endif

            dprlib_uninit_dpram(&card->cp, DPR_ERR_HOST_SHUTDOWN_REQUEST);
            if ( DPRLIB_STOP == work ) {
                DPRLIBLOGMSG(" <DPRLIB_STOP: daemon done \n");
                break;
            }

            /* do cp16xx hw reset */
            {
                volatile DPR_VOID * reg_reset_addr;
                volatile DPR_UINT32 reset_value;

                cp16xx_os_irq_uninit(card);
                DPRLIBLOGMSG("ASIC kennung %x\n", DPR_READ_UINT32(card->bars[PCI_BAR_BOOTROM].bar_ptr + ASIC_ID_OFFSET));

                reg_reset_addr = card->bars[PCI_BAR_BOOTROM].bar_ptr + ASIC_RESET_REQ_OFFSET;
                reset_value = DPR_READ_UINT32(reg_reset_addr);
                reset_value |= 2;
                DPR_WRITE_UINT32( reset_value, reg_reset_addr);
            }

            /* pci initialize on card is slow */
            DPR_TASK_DELAY_UNINTERRUPTIBLE(2000);

            DPR_ASSERT(sizeof(tmp) > cp16xx_os_get_info(card, tmp, sizeof(tmp)));
            DPRLIBINFMSG("reset card: %s %s\n",
                         tmp, DPR_READ_UINT32(card->bars[PCI_BAR_BOOTROM].bar_ptr + ASIC_RESET_ACK_OFFSET) & 0x2 ?
                         "successful" : "failed");

            cp16xx_os_irq_init(card);
            fw_start_counter = 0;
            cp16xx_daemon_work(card, DPRLIB_START);
            DPRLIBLOGMSG(" <DPRLIB_RESET_FW: daemon done \n");
        } else if ( DPRLIB_START == work ) {
            /* now try to contact the firmware */
            DPRLIBLOGMSG("DPRLIB_START: wait for FW to be ready fw-loop-count=%d \n", fw_start_counter);
            if ( DPR_READ_UINT32(&card->cp.pDpramConfigArea->SPIConfig) != DPR_INT_CP_START &&
                 DPR_READ_UINT32(&card->cp.pDpramConfigArea->SPIConfig) != DPR_INT_DPR_READY ) {
                DPRLIBLOGMSG("FW is not ready, repeat 200 ms later\n");
                DPR_TIMER_START(&connect_timer, 200);
                fw_start_counter++;
                continue;
            }
            DPRLIBLOGMSG("DPRLIB_START: FW ready: loop-count=%d \n", fw_start_counter);

            cp16xx_irq_restore_mask_rt(card);
            cp16xx_irq_restore_mask_irt(card);

            if ( DPR_OK == DPRLIB_start(&card->cp) ) {
                dprlib_start_counter = 0;  /* debug loop counter */
                DPR_ASSERT(sizeof(tmp) > cp16xx_os_get_info(card, tmp, sizeof(tmp)));
                DPRLIBINFMSG("DPRLIB_START: successfully connect firmware on card: %s\n", tmp);

                DPR_ASSERT(sizeof(tmp) > DPRLIB_get_remote_version_as_string_ext(&card->cp, tmp, sizeof(tmp)));
                DPRLIBINFMSG("FW version %s\n", tmp);

                if ( card->watchdog_cycle ) {
                    /* FW observes driver, we use app structure for hold it */
                    struct cp16xx_app_data *fw_observes_drv;
                    DPRLIBLOGMSG("create watchdog: wd-cycle=%d >cp16xx_app_new... \n", card->watchdog_cycle);
                    fw_observes_drv = cp16xx_app_new(card, DRIVER_WD);
                    fw_observes_drv->wd_timeout = card->watchdog_cycle;
                    fw_observes_drv->wd_owner = WD_FLAG_FIRMWARE;
                    fw_observes_drv->wd_user = WD_USER_REF_HOSTDRIVER;
                    if ( cp16xx_wd_activate(fw_observes_drv) ) {
                        DPR_ASSERT(sizeof(tmp) > cp16xx_os_get_info(card, tmp, sizeof(tmp)));
                        DPRLIBERRMSG("cannot setup driver watchdogs on card: %s\n", tmp);
                        cp16xx_app_free(card, fw_observes_drv);
                    } else {
                        /* ok, watchdog activated */
#ifdef CP16XX_COMBINED_DRIVER
                        card->card_wd_app = fw_observes_drv;
#endif
                        DPRLIBLOGMSG("watchdog ok done \n");
                    }
                }
#ifdef CP16XX_COMBINED_DRIVER
                /* confusing function name: we are signaling card start */
                DPRLIBLOGMSG("start ok: ->cp16xx_os_reset_end ...\n");
                cp16xx_os_reset_end(card);
                DPRLIBLOGMSG("start ok: <-cp16xx_os_reset_end done\n");
#endif
            } else {
                DPR_ASSERT(sizeof(tmp) > cp16xx_os_get_info(card, tmp, sizeof(tmp)));
                DPRLIBINFMSG("DPRLIB_START: unsuccessfull attempt to connect firmware on card: %s, repeat\n", tmp);
                DPRLIBINFMSG("DPRLIB_START: dprlib_start_counter= %d \n", dprlib_start_counter);
                dprlib_start_counter++;
                DPR_SEM_POST(card->daemon_resume);
            }
        }
    }

    DPR_TIMER_DELETE_SYNC(&connect_timer);

    card->daemon_handle = 0;

    DPRLIBLOGMSG("end ok \n");

    return 0;
} /* end of cp16xx_daemon */

/* cp16xx_dprlib_setup - simple helper, registers callbacks for incoming packets */
void cp16xx_dprlib_setup(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

    /* bind callbacks */
    card->cp.trigger_irq = cp16xx_irq_trigger;
    card->cp.wakeup_daemon = cp16xx_daemon_work;
    card->cp.parent = card;

    DPRLIB_channel_register_cbf(&card->cp, DPR_PNIO_CMD_CHN, cp16xx_base_read_cbf);
    DPRLIB_channel_register_cbf(&card->cp, DPR_PNIO_ALARM_CHN, cp16xx_base_read_cbf);
    DPRLIB_channel_register_cbf(&card->cp, DPR_PNIO_MOD_CHN, cp16xx_base_read_cbf);
    DPRLIB_channel_register_cbf(&card->cp, DPR_PNIO_DATA_REC_CHN, cp16xx_base_read_cbf);
    DPRLIB_channel_register_cbf(&card->cp, DPR_MGT_CHN, cp16xx_base_read_cbf);
    DPRLIB_channel_register_cbf(&card->cp, DPR_L2_SEND_CHN, cp16xx_base_read_cbf);
    DPRLIB_channel_register_cbf(&card->cp, DPR_L2_RECV_CHN, cp16xx_base_read_cbf);
    /* bind callbacks END *     */

    DPRLIBLOGMSG("end\n");
}

/* cp16xx_dma_(init|uninit) allocs memory for DMA operation, */
/*   porting is given over macros substitution in os_*.h */
int cp16xx_dma_init(struct cp16xx_card_data *card)
{
    int order;

    DPRLIBLOGMSG("begin\n");

    /* try to get host dma pages... if not reduce page order and try again */
    /* irt and l2 are allocated contiguous, irt first */
    card->irt.size = MAX_SIZE_DMA_IMAGE;
    for ( order = MAX_L2ETH_DMA_PAGE_ORDER; order >= MIN_L2ETH_DMA_PAGE_ORDER; order-- ) {
        card->l2.size = (1 << order) * PAGE_SIZE;
        card->irt.virt = DPR_ALLOC_CONSISTENT_DMA(card, (card->irt.size + card->l2.size), &card->irt.phys);
        if ( card->irt.virt ) {
            if ( order < MAX_L2ETH_DMA_PAGE_ORDER )
                DPRLIBINFMSG("could allocate only (order %i) DMA memory l2 than (order %i)\n",
                             order, MAX_L2ETH_DMA_PAGE_ORDER);

            card->l2.virt = (void *)((char *)card->irt.virt + card->irt.size);
            DPR_PHYSICAL_ADDRESS(card->l2.phys) = DPR_PHYSICAL_ADDRESS(card->irt.phys) + card->irt.size;
            memset(card->l2.virt, 0x00, card->l2.size); /* clean dma memory */

            break; /* got the requested size */
        }
    }

    /* if no contiguous memory for irt plus l2 available, allocate irt only */
    if ( !card->irt.virt ) {
        card->irt.virt = DPR_ALLOC_CONSISTENT_DMA(card, card->irt.size, &card->irt.phys);
    }

    if ( card->irt.virt ) {
        DPRLIBLOGMSG("got PNIO phys 0x%p, virt 0x%p, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_PTR(card->irt.phys), card->irt.virt, card->irt.size);
        DPR_ATOMIC_SET(card->irt.count, 0);
    } else {
        DPRLIBERRMSG("###### failed  DPR_ALLOC_CONSISTENT_DMA(IRT size %lu)\n", card->irt.size);
        goto cp16xx_dma_init_fail;
    }

    if ( card->l2.virt ) {
        DPRLIBLOGMSG("got L2 phys 0x%p, virt 0x%p, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_PTR(card->l2.phys), card->l2.virt, card->l2.size);
        DPR_ATOMIC_SET(card->l2.count, 0);
    } else {
        DPRLIBERRMSG("###### failed  __get_dma_pages(order >= MIN_L2ETH_DMA_PAGE_ORDER), disabled L2\n");
    }

#ifdef SWITCH_DIAG_TRACE

    /* alloc DMA-able memory for ertec switch port diagnostic trace buffer */
    card->trc.size = 0x400 + (4 * 2) /* TCW */;
    card->trc.virt = DPR_ALLOC_CONSISTENT_DMA(card, card->trc.size, &card->trc.phys);
    if ( card->trc.virt ) {
        DPRLIBLOGMSG("got TRACE phys 0x%p, virt 0x%p, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_PTR(card->trc.phys), card->trc.virt, card->trc.size);
        DPR_ATOMIC_SET(card->trc.count, 0);
    } else {
        DPRLIBERRMSG("###### failed  DPR_ALLOC_CONSISTENT_DMA(TRC size %lu)\n", card->trc.size);
        goto cp16xx_dma_init_fail;
    }

#endif /* SWITCH_DIAG_TRACE */

    return 0;

    cp16xx_dma_init_fail:

    return E_CP16XX_NOMEMORY;
}

int cp16xx_dma_uninit(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

#ifdef SWITCH_DIAG_TRACE
    if ( card->trc.virt )
        DPR_FREE_CONSISTENT_DMA(card, card->trc.size, card->trc.virt, card->trc.phys);
#endif /* SWITCH_DIAG_TRACE */

    /* irt and l2 are allocated contiguous, irt first */
/*  if(card->l2.virt) */
/*      DPR_FREE_CONSISTENT_DMA(card, card->l2.size, card->l2.virt, card->l2.phys); */

    if ( card->irt.virt )
        DPR_FREE_CONSISTENT_DMA(card, card->irt.size, card->irt.virt, card->irt.phys);

    DPRLIBLOGMSG("end\n");

    return 0;
}

/* cp16xx_pci_(init|uninit) blend IO memory in driver space,
   porting is given over macros substitution in os_*.h */
int cp16xx_pci_init(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

    card->bars[PCI_BAR_BOOTROM].bar_ptr = DPR_IOREMAP_NOCACHE(card,
                                                              card->bars[PCI_BAR_BOOTROM].bar_base_physaddr, card->bars[PCI_BAR_BOOTROM].bar_size);
    if ( card->bars[PCI_BAR_BOOTROM].bar_ptr ) {
        DPRLIBLOGMSG("virtual address bootrom_bar_ptr 0x%p\n", card->bars[PCI_BAR_BOOTROM].bar_ptr);
    } else {
        DPRLIBERRMSG("impossible to ioremap bootrom pci bar start 0x%lx, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_ULONG(card->bars[PCI_BAR_BOOTROM].bar_base_physaddr),
                     card->bars[PCI_BAR_BOOTROM].bar_size);
        goto cp16xx_pci_init_fail;
    }

    /* register base address for initialization
       ioremap_nocache - a device driver can access any I/O memory address,
       whether it is directly mapped to virtual address space or not.
       these addresses should not be dereferenced directly; instead,
       functions like readb/DPR_READ_UINT32 should be used. */
    card->bars[PCI_BAR_IRTE].bar_ptr = DPR_IOREMAP_NOCACHE(card,
                                                           card->bars[PCI_BAR_IRTE].bar_base_physaddr, card->bars[PCI_BAR_IRTE].bar_size);
    if ( card->bars[PCI_BAR_IRTE].bar_ptr ) {
        DPRLIBLOGMSG("virtual address register_bar_ptr 0x%p\n", card->bars[PCI_BAR_IRTE].bar_ptr);
    } else {
        DPRLIBERRMSG("impossible to ioremap register pci bar start 0x%lx, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_ULONG(card->bars[PCI_BAR_IRTE].bar_base_physaddr),
                     card->bars[PCI_BAR_IRTE].bar_size);
        goto cp16xx_pci_init_fail;
    }

    /* following mapping for the onboard flash is important for the FlashRescue prgram to work */
    card->bars[PCI_BAR_EMIF_CS0].bar_ptr = DPR_IOREMAP_NOCACHE(card,
                                                               card->bars[PCI_BAR_EMIF_CS0].bar_base_physaddr, card->bars[PCI_BAR_EMIF_CS0].bar_size);
    if ( card->bars[PCI_BAR_EMIF_CS0].bar_ptr ) {
        DPRLIBLOGMSG("virtual address emif_bar_ptr 0x%p\n", card->bars[PCI_BAR_EMIF_CS0].bar_ptr);
    } else {
        DPRLIBERRMSG("impossible to ioremap emif pci bar start 0x%lx, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_ULONG(card->bars[PCI_BAR_EMIF_CS0].bar_base_physaddr),
                     card->bars[PCI_BAR_EMIF_CS0].bar_size);
        goto cp16xx_pci_init_fail;
    }

    card->bars[PCI_BAR_DPRAM].bar_ptr = DPR_IOREMAP_NOCACHE(card,
                                                            card->bars[PCI_BAR_DPRAM].bar_base_physaddr, card->bars[PCI_BAR_DPRAM].bar_size);
    if ( card->bars[PCI_BAR_DPRAM].bar_ptr ) {
        DPRLIBLOGMSG("virtual address dpram_bar_ptr 0x%p\n", card->bars[PCI_BAR_DPRAM].bar_ptr);
    } else {
        DPRLIBERRMSG("impossible to ioremap dpram pci bar start 0x%lx, size 0x%lx\n",
                     DPR_PHYSICAL_ADDRESS_TO_ULONG(card->bars[PCI_BAR_DPRAM].bar_base_physaddr),
                     card->bars[PCI_BAR_DPRAM].bar_size);
        goto cp16xx_pci_init_fail;
    }

    card->cp.pDpramConfigArea = (PDPR_BASE)(card->bars[PCI_BAR_DPRAM].bar_ptr +
                                            DPR_CONFIG_AREA_OFFSET);
    DPRLIBLOGMSG("virtual address dpram_config_area_ptr 0x%p\n", card->cp.pDpramConfigArea);

    DPRLIBLOGMSG("swi_version=0x%08x(must be 0x2005xxxx)\n",
                 DPR_READ_UINT32((card->bars[PCI_BAR_IRTE].bar_ptr + SWI_VERSION)));


    DPRLIBLOGMSG("end\n");

    return 0;

    cp16xx_pci_init_fail:

    return E_CP16XX_NOMEMORY;
}

int cp16xx_pci_uninit(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

    if ( card->bars[PCI_BAR_DPRAM].bar_ptr )
        DPR_IOUNMAP(card, card->bars[PCI_BAR_DPRAM].bar_ptr, card->bars[PCI_BAR_DPRAM].bar_size);
    if ( card->bars[PCI_BAR_IRTE].bar_ptr )
        DPR_IOUNMAP(card, card->bars[PCI_BAR_IRTE].bar_ptr, card->bars[PCI_BAR_IRTE].bar_size);
    if ( card->bars[PCI_BAR_EMIF_CS0].bar_ptr )
        DPR_IOUNMAP(card, card->bars[PCI_BAR_EMIF_CS0].bar_ptr, card->bars[PCI_BAR_EMIF_CS0].bar_size);
    if ( card->bars[PCI_BAR_BOOTROM].bar_ptr )
        DPR_IOUNMAP(card, card->bars[PCI_BAR_BOOTROM].bar_ptr, card->bars[PCI_BAR_BOOTROM].bar_size);

    DPRLIBLOGMSG("end\n");

    return 0;
}

/*-----------------------------------------------------------------------------
 * Name  : cp16xx_dma_baseaddr_set
 * Descr : Sets DMA destination addr. in the firmware. See also cp16xx_dma_init()
 *         DMA transfer is used for IRT Data and L2 frames (L2 == Ndis for Windows)
 * Note:   Is called only by windows driver:\driver\windows\mp16xx_windows.c
 * Param :
 *  [ IN]: cp16xx_card_data *card: HW board specific parameter structure
 * Return: ==0: ok, != 0 error
 */
int cp16xx_dma_baseaddr_set(struct cp16xx_card_data *card)
{
    struct cp16xx_app_data * tmp_app_ptr;
    DPRMGTCHNL_TO_MGT_SET_DMA dma_msg;
    unsigned long l2_base;
    unsigned long irt_base;
    int iret=0, res;

    DPRLIBLOGMSG("begin: calling cp16xx_app_new: type=%d == DRIVER_INTERNAL ... \n",DRIVER_INTERNAL);

    tmp_app_ptr = cp16xx_app_new(card, DRIVER_INTERNAL);

    if ( NULL == tmp_app_ptr ) {
        iret = -1;
        DPRLIBERRMSG("failed to register temporary application\n");
    }

    if ( 0 == iret ) {
        dma_msg.rqb.header.hostref = CPU_TO_LE(tmp_app_ptr->user_id);
        dma_msg.rqb.header.subsystem = FW_SUBSYS_CP_MGT;
        dma_msg.rqb.header.userid = CPU_TO_LE(0x99999999);
        dma_msg.rqb.header.response = CPU_TO_LE(0);
        dma_msg.rqb.header.userdatalength = CPU_TO_LE(sizeof(dma_msg.rqb.mgt_opcode) + sizeof(void *));
        dma_msg.rqb.mgt_opcode = CPU_TO_LE16(OPC_MGT_HOST_SET_DMA_REQ);

        l2_base = DPR_PTR_TO_ULONG(DPR_PHYSICAL_ADDRESS_TO_PTR(card->l2.phys));
        irt_base = DPR_PTR_TO_ULONG(DPR_PHYSICAL_ADDRESS_TO_PTR(card->irt.phys));
        dma_msg.base = (uint32_t)((l2_base < irt_base) ? l2_base : irt_base);

        DPRLIBLOGMSG("irt_base 0x%08lx, l2_base 0x%08lx\n", irt_base, l2_base);

        tmp_app_ptr->mgt_synch_data = &dma_msg;

        DPRLIBLOGMSG("send dma base addr=0x%04x for appl-id=%lu  opc=%#x \n", dma_msg.base, tmp_app_ptr->user_id, OPC_MGT_HOST_SET_DMA_REQ);

        if ( DPR_OK != DPRLIB_channel_write_message(&card->cp, DPR_MGT_CHN, 0, &dma_msg, sizeof(dma_msg)) ) {
            iret = -2;
            tmp_app_ptr->mgt_synch_data = NULL;
            DPRLIBERRMSG("failed to send over mgt channel\n");
        }
    }

    if ( 0 == iret ) {
        res = DPR_SEM_WAIT_FOR_WAKEUP(tmp_app_ptr->mgt_synch, 4000);
        if ( 0 != res ) {
            iret = -3;
            tmp_app_ptr->mgt_synch_data = NULL;
            DPRLIBERRMSG("got a signal or timeout res=%d (-1=interrupt, 1=timeout) \n",res);
        }
    }

    if ( 0 == iret ) {
        if ( LE_TO_CPU16(dma_msg.rqb.mgt_opcode) == OPC_MGT_HOST_SET_DMA_CNF ) {
            /* confirmation opcode ok */
            if ( dma_msg.rqb.header.response ) {
                DPRLIBERRMSG("got an invalid response=0x%x\n", LE_TO_CPU(dma_msg.rqb.header.response));
                iret = -4;
            } else {
                DPRLIBLOGMSG("confirmation OK, appl-id=%lu opc=%#x \n", tmp_app_ptr->user_id, OPC_MGT_HOST_SET_DMA_CNF);
            }
        }
        else {
            DPRLIBERRMSG("invalid opcode=%#x, expected=%#x \n", LE_TO_CPU16(dma_msg.rqb.mgt_opcode), OPC_MGT_HOST_SET_DMA_CNF );
            iret = -5;
        }
    }

    if ( NULL != tmp_app_ptr ) {
        cp16xx_app_free(card, tmp_app_ptr);
    }

    DPRLIBLOGMSG("end: iret=%d (0=OK)\n",iret);
    return iret;
} /* end of cp16xx_dma_baseaddr_set */


/* helper for card structure handling

   cp16xx_card_ref_get - get structure pointer from index

   cp16xx_card_ref_add - add structure to cp16xx_cards array(protected with cp16xx_cards_lock)

   cp16xx_card_ref_del - remove structure from cp16xx_cards array

   cp16xx_card_init - initialize common elements of structure

   cp16xx_card_uninit - destroy common elements of structure */
struct cp16xx_card_data *cp16xx_card_ref_get(int i)
{
    if ( i < 0 || i >= MAX_CP16XX_DEVICES )
        return NULL;

    return cp16xx_cards[i];
}

static int cp16xx_card_ref_add(struct cp16xx_card_data *card)
{
    int i;
    DPR_SPINLOCK_FLAGS flags;

    /* queue private data in global static array of this */
    DPR_SPINLOCK_LOCK(cp16xx_cards_lock, flags);
    for ( i = 0; i < MAX_CP16XX_DEVICES; i++ ) {
        if ( !cp16xx_cards[i] ) {
            cp16xx_cards[i] = card;
            card->cp_index = i;
            break;
        }
    }
    DPR_SPINLOCK_UNLOCK(cp16xx_cards_lock, flags);

    if ( i >= MAX_CP16XX_DEVICES ) {
        DPRLIBERRMSG("too much CPs\n");
        return E_CP16XX_MAXREACHED;
    }

    return i;
}

static void cp16xx_card_ref_del(struct cp16xx_card_data *card)
{
    int i;
    DPR_SPINLOCK_FLAGS flags;

    DPR_SPINLOCK_LOCK(cp16xx_cards_lock, flags);
    for ( i = 0; i < MAX_CP16XX_DEVICES; i++ ) {
        if ( cp16xx_cards[i] == card ) {
            DPR_ASSERT(card->cp_index == i);
            cp16xx_cards[i] = NULL;
            break;
        }
    }
    DPR_SPINLOCK_UNLOCK(cp16xx_cards_lock, flags);
}

int cp16xx_card_init(struct cp16xx_card_data *card)
{
    int err, ret, i, j;

    DPRLIBLOGMSG("begin\n");

    DPR_SPINLOCK_INIT(card->smplock);

    DPR_INIT_WAIT_QUEUE_RT(card->rt_irq);
    DPR_INIT_WAIT_QUEUE_IRT(card->startop_irq);
    DPR_INIT_WAIT_QUEUE_IRT(card->opfault_irq);
    DPR_INIT_WAIT_QUEUE_IRT(card->newcycle_irq);

    DPR_CARD_INIT_OS(card);
    for ( i = 0;  i < MAX_APPLICATIONS; ++i ) {
        DPR_APP_INIT_OS(&card->apps[i]);
        card->apps[i].user_id = FREE_ID;
        card->apps[i].type = NOT_USED;
        DPR_SEM_CREATE(card->apps[i].mgt_synch);
        for ( j = 0; j < DPR_CFG_MAX_CHANNEL; ++j ) {
            DPR_CHANNEL_INIT_OS(&card->apps[i].pools[j]);
            card->apps[i].pools[j].channel_number = j;
            card->apps[i].pools[j].user_id = FREE_ID;
            DPR_MUTEX_CREATE_UNLOCKED(card->apps[i].pools[j].wmutex);
        }
    }

    DPR_SPINLOCK_INIT(card->apps_lock);
    DPR_ATOMIC_SET(card->applications_count, 0);

    DPR_SEM_CREATE(card->daemon_resume);

#ifdef CP16XX_COMBINED_DRIVER
    DPR_SEM_CREATE(card->dprstart_done);
    card->card_wd_app = NULL;
#endif

    DPR_ATOMIC_SET(card->daemon_work, DPRLIB_START);

    card->watchdog_cycle = watchdog_cycle;
    DPR_ATOMIC_SET(card->applications_watchdogs, 0);

    card->startopfct = cp16xx_irt_startop_cbf_default;
    card->opfaultfct = cp16xx_irt_opfault_cbf_default;
    card->newcyclefct = cp16xx_irt_newcycle_cbf_default;

    cp16xx_dprlib_setup(card);

    /* queue private data in global static array of this */
    err = cp16xx_card_ref_add(card);
    if ( err < 0 )
        ret = E_CP16XX_FAULT;
    else
        ret = 0;

    DPRLIBLOGMSG("end\n");

    return ret;
}

void cp16xx_card_uninit(struct cp16xx_card_data *card)
{
    int i, j;

    DPRLIBLOGMSG("begin\n");

    cp16xx_card_ref_del(card);

    DPR_SEM_DESTROY(card->daemon_resume);

    DPR_SPINLOCK_UNINIT(card->apps_lock);

    for ( i = 0;  i < MAX_APPLICATIONS; ++i ) {
        for ( j = 0; j < DPR_CFG_MAX_CHANNEL; ++j ) {
            DPR_MUTEX_DESTROY(card->apps[i].pools[j].wmutex);
            DPR_CHANNEL_UNINIT_OS(&card->apps[i].pools[j]);
        }
        DPR_SEM_DESTROY(card->apps[i].mgt_synch);
        DPR_APP_UNINIT_OS(&card->apps[i]);
    }
    DPR_CARD_UNINIT_OS(card);

    DPR_UNINIT_WAIT_QUEUE_RT(card->rt_irq);
    DPR_UNINIT_WAIT_QUEUE_IRT(card->startop_irq);
    DPR_UNINIT_WAIT_QUEUE_IRT(card->opfault_irq);
    DPR_UNINIT_WAIT_QUEUE_IRT(card->newcycle_irq);

    DPR_SPINLOCK_UNINIT(card->smplock);

    DPRLIBLOGMSG("end\n");
}
