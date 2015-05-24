/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx_linux_irq.c
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

#include "../cp16xx_base.h"    /* common (kernel side only) structures and defines */

/* IRQ related functions(OS dependent)

   cp16xx_irq_shared_cbf - ISR

   cp16xx_os_irq_init - install and connect ISR

   cp16xx_os_irq_uninit - disconnect and remove ISR */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t cp16xx_irq_shared_cbf(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t cp16xx_irq_shared_cbf(int irq, void *dev_id)
#endif
{
    u32 irtIrq, rtIrq, irq_handled = 0;
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)dev_id;
    DPR_SPINLOCK_FLAGS flags;

    DPR_SPINLOCK_LOCK(card->smplock, flags);

    irtIrq = DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_IRT);
    if(irtIrq) {
        #ifdef DBG_COUNTERS
            card->irt_irq_count++;
        #endif /* DBG_COUNTERS */
        ++irq_handled;

        /* clear the interrupting bit */
        DPR_WRITE_UINT32(irtIrq, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ_ACK_IRT);
    }

    rtIrq = DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_RT);
    if(rtIrq) {
        #ifdef DBG_COUNTERS
            card->rt_irq_count++;
        #endif /* DBG_COUNTERS */
        ++irq_handled;

        /* clear the interrupting bit */
        DPR_WRITE_UINT32(rtIrq, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ_ACK_RT);
    }

    /* signal end of irq, propably spurios interrupt from ERTEC */
    DPR_WRITE_UINT32(0x0000000F, card->bars[PCI_BAR_IRTE].bar_ptr + HP_EOI_IRQ1);

    DPR_SPINLOCK_UNLOCK(card->smplock, flags);

    if(irtIrq) {
        if(irtIrq & OPFAULT_BITS)
            card->opfaultfct(card);

        if(irtIrq & BIT_IRQ_STARTOP)
            card->startopfct(card);

        if(irtIrq & BIT_IRQ_NEWCYCLE) {
            #ifdef DBG_COUNTERS
                card->newcyc_irq_count++;
            #endif /* DBG_COUNTERS */
            card->newcyclefct(card);
        }
    }
    if(rtIrq) {
        DPR_WAKEUP_QUEUE(card->rt_irq);
        dprlib_int_callback(&card->cp);
    }

    if(irq_handled) {
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}

int cp16xx_os_irq_init(struct cp16xx_card_data *card)
{
    int err = -EFAULT;
    DPRLIBLOGMSG("begin\n");

    DPRLIBLOGMSG("request irq %i, card 0x%p\n", card->os_info.irq, card);
    if(card->os_info.irq >= 0) {
        if((err = request_irq(card->os_info.irq, cp16xx_irq_shared_cbf,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
            SA_SHIRQ | SA_INTERRUPT,
#else
            IRQF_SHARED,
#endif
            "cp16xx", card))) {

            DPRLIBERRMSG("can't get assigned irq %i, error %i\n", card->os_info.irq, err);
            return err;
        }
        DPRLIBLOGMSG("request irq %i  -> OK\n", card->os_info.irq);
    }

  #ifdef DBG_COUNTERS
    card->newcyc_irq_count = 0;
    card->rt_irq_count = 0;
    card->irt_irq_count = 0;
  #endif /* DBG_COUNTERS */

    cp16xx_irq_set_mode(card);
    cp16xx_irq_restore_mask_rt(card);
    cp16xx_irq_restore_mask_irt(card);

    DPRLIBLOGMSG("end\n");

    return 0;
}

void cp16xx_os_irq_uninit(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

    cp16xx_irq_reset_mask_irt(card);
    cp16xx_irq_reset_mask_rt(card);

    if(card->os_info.irq >= 0) {
        free_irq(card->os_info.irq, card);
    }

    DPRLIBLOGMSG("end\n");
}

static ssize_t cp16xx_os_syncronize_irt(struct cp16xx_interface *interface, size_t count)
{
    int ret = 0;
    DPR_WAIT_POINT wait;
    DPRLIBLOGMSG("begin\n");

    DPR_PREPARE_WAIT_ON_POINT(wait);
    if(count == CP16XX_STARTOP) {
        DPR_WAIT_ON_POINT(interface->card->startop_irq, wait);
    } else if(count == CP16XX_OPFAULT) {
        DPR_WAIT_ON_POINT(interface->card->opfault_irq, wait);
    } else if(count == CP16XX_NEWCYCLE) {
        DPR_WAIT_ON_POINT(interface->card->newcycle_irq, wait);
    }

    schedule();

    if(count == CP16XX_STARTOP) {
        DPR_REMOVE_WAIT_POINT(interface->card->startop_irq, wait);
    } else if(count == CP16XX_OPFAULT) {
        DPR_REMOVE_WAIT_POINT(interface->card->opfault_irq, wait);
    } else if(count == CP16XX_NEWCYCLE) {
        DPR_REMOVE_WAIT_POINT(interface->card->newcycle_irq, wait);
    }

    DPRLIBLOGMSG("end\n");

    return ret;
}
