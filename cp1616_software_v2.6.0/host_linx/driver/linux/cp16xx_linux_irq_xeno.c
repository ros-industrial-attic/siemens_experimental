/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx_linux_irq_rtai.c
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

#ifdef XENOMAI

/* IRQ related functions(OS dependent)

   cp16xx_sh_interrupt_rt - Xenomai ISR

   cp16xx_os_irq_init - install and connect ISR

   cp16xx_os_irq_uninit - disconnect and remove ISR */
static irqreturn_t cp16xx_sh_interrupt_rt(xnintr_t *arg)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)(I_DESC(arg)->private_data);
    u32 irtIrq, rtIrq, irq_handled = 0;

    irtIrq = DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_IRT);
    if(irtIrq) {
        ++irq_handled;

        if(irtIrq & OPFAULT_BITS)
            card->opfaultfct(card);

        if(irtIrq & BIT_IRQ_STARTOP)
            card->startopfct(card);

        if(irtIrq & BIT_IRQ_NEWCYCLE)
            card->newcyclefct(card);

        /* clear the interrupting bit */
        DPR_WRITE_UINT32(irtIrq, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ_ACK_IRT);
    }

    rtIrq = DPR_READ_UINT32(card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ1_RT);
    if(rtIrq) {
        ++irq_handled;

        rt_nrtsig_pend(&card->os_info.rt_tasklet);

        /* clear the interrupting bit */
        DPR_WRITE_UINT32(rtIrq, card->bars[PCI_BAR_IRTE].bar_ptr + HP_IRQ_ACK_RT);
    }

    /* signal end of irq, propably spurios interrupt from ERTEC */
    DPR_WRITE_UINT32(0x0000000F, card->bars[PCI_BAR_IRTE].bar_ptr + HP_EOI_IRQ1);

    if(irq_handled) {
        return RT_INTR_HANDLED;
    } else {
        static int i = 0;
        if(i < 10) {
            ++i; /* do not pollute syslog */
            DPRLIBERRMSG("Wrong interrupt configuration for Realtime IRQ %d. Interrupt must be exclusive.\n",
                card->os_info.irq);
        }
        return RT_INTR_NONE;
    }
}

static void cp16xx_tasklet_handler(rt_nrtsig_t rt_sig, void *cookie)
{
    struct cp16xx_card_data *card = (struct cp16xx_card_data *)(cookie);

    DPR_WAKEUP_QUEUE(card->rt_irq);
    dprlib_int_callback(&card->cp);
}

int cp16xx_os_irq_init(struct cp16xx_card_data *card)
{
    int err = -EFAULT;
    DPRLIBLOGMSG("begin\n");

    DPRLIBLOGMSG("request irq %i, card 0x%p\n", card->os_info.irq, card);
    if(card->os_info.irq >= 0) {
        char tmp[200], *ptmp;

        ptmp = XENO_UNIC_NAME(tmp, XENO_MAGIC_STARTOP, card->cp_index);
        if((err = rt_sem_create(&card->startop_irq, ptmp, 0, S_PULSE))) {
            DPRLIBERRMSG("can't create startop %s semaphore\n", ptmp);
            goto cp16xx_init_irq_fail_startop;
        }
        ptmp = XENO_UNIC_NAME(tmp, XENO_MAGIC_OPFAULT, card->cp_index);
        if((err = rt_sem_create(&card->opfault_irq, ptmp, 0, S_PULSE))) {
            DPRLIBERRMSG("can't create opfault %s semaphore\n", ptmp);
            goto cp16xx_init_irq_fail_opfault;
        }
        ptmp = XENO_UNIC_NAME(tmp, XENO_MAGIC_NEWCYCLE, card->cp_index);
        if((err = rt_sem_create(&card->newcycle_irq, ptmp, 0, S_PULSE))) {
            DPRLIBERRMSG("can't create newcycle %s semaphore\n", ptmp);
            goto cp16xx_init_irq_fail_newcycle;
        }

        if((err = rt_nrtsig_init(&card->os_info.rt_tasklet, cp16xx_tasklet_handler, card)))
            goto cp16xx_init_softirq_fail;

        /* xenomai accepts only unic name for interrupt handling */
        ptmp = XENO_UNIC_NAME(tmp, "cp1616", card->cp_index);
        if((err = rt_intr_create(&card->os_info.rt_intr, ptmp,
            card->os_info.irq, cp16xx_sh_interrupt_rt, NULL, I_SHARED)))
            goto cp16xx_init_irq_irt_fail;

        card->os_info.rt_intr.private_data = card;
        rt_intr_enable(&card->os_info.rt_intr);

        DPRLIBLOGMSG("request irq %i  -> OK\n", card->os_info.irq);
    }

    cp16xx_irq_set_mode(card);
    cp16xx_irq_restore_mask_rt(card);
    cp16xx_irq_restore_mask_irt(card);

    DPRLIBLOGMSG("end\n");

    return 0;

cp16xx_init_irq_irt_fail:
    rt_nrtsig_destroy(&card->os_info.rt_tasklet);
cp16xx_init_softirq_fail:
    rt_sem_delete(&card->newcycle_irq);
cp16xx_init_irq_fail_newcycle:
    rt_sem_delete(&card->opfault_irq);
cp16xx_init_irq_fail_opfault:
    rt_sem_delete(&card->startop_irq);
cp16xx_init_irq_fail_startop:

    DPRLIBERRMSG("can't get assigned irq %i, error %i\n", card->os_info.irq, err);

    return err;
}

void cp16xx_os_irq_uninit(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

    cp16xx_irq_reset_mask_irt(card);
    cp16xx_irq_reset_mask_rt(card);

    if(card->os_info.irq >= 0) {
        rt_intr_disable(&card->os_info.rt_intr);
        rt_intr_delete(&card->os_info.rt_intr);
        rt_nrtsig_destroy(&card->os_info.rt_tasklet);
        rt_sem_delete(&card->opfault_irq);
        rt_sem_delete(&card->startop_irq);
        rt_sem_delete(&card->newcycle_irq);
    }

    DPRLIBLOGMSG("end\n");
}

static ssize_t cp16xx_os_syncronize_irt(struct cp16xx_interface *interface, size_t count)
{
    int ret = -EFAULT;
    DPRLIBERRMSG("compiled with XENOMAI, don't use read interface for synchronisation\n");

    return ret;
}

#endif /* XENOMAI */
