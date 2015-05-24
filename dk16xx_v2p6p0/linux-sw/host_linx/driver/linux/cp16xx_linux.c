/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx_linux.c
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

#include "../fw1616dk_vers.h"  /* fw version constant FW_VER_MAJOR, FW_VER_MINOR1... */
#include "../driver_vers.h"    /* driver version constants */

#include "../dprintern.c"      /* common layer for FW and driver */
#include "../dprlibhost.c"     /* host specific extension of common layer */
#include "../cp16xx_base.c"    /* host specific functions */

#ifdef NETWORK_DRIVER
#include "cp16xx_linux_net.c"
#endif
/* driver version number string: e.g.: '1.1.3.4' First 2 digits are the product version, last digit
 *                               is a build number. For driver version see driver_vers.h
 */
#define ch2STR(x)  #x
#define CH2STR(x) ch2STR(x)
#if defined(RTAI)
 #define RT_EXTENSION " (with RTAI support)"
 #include "cp16xx_linux_irq_rtai.c"
#elif defined(XENOMAI)
 #define RT_EXTENSION " (with XENOMAI support)"
 #include "cp16xx_linux_irq_xeno.c"
#else
 #define RT_EXTENSION
 #include "cp16xx_linux_irq.c"
#endif
#define cp16xx_driver_version CH2STR(FW_VER_MAJOR) "."  CH2STR(FW_VER_MINOR1) "." CH2STR(DRV_VER_MINOR) "." \
                              CH2STR(VERSION_BUILD_NUMBER)RT_EXTENSION

/* Linux modules provide interface for parameters, use it */
extern int watchdog_cycle;
static int major = 0; /* 0 - means Linux takes first free major, usually 255,
 override it or use module param by loading */

MODULE_DESCRIPTION("Example for SIEMENS CP1604, CP1616 PROFINET CP");
MODULE_LICENSE("Copyright 2005-2007, SIEMENS AG");
MODULE_AUTHOR("Alexander Kern");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
MODULE_VERSION(cp16xx_driver_version);
#endif

module_param(major, int, S_IRUGO);
MODULE_PARM_DESC(major, "major number, 0 - get it automatically");

module_param(watchdog_cycle, int, S_IRUGO);
MODULE_PARM_DESC(watchdog_cycle, "FW watchdog timeout, 0 - disable");

/* wrap OS independent callback in OS dependent timer callback */
void DPR_LINUX_TIMER_FCT(unsigned long arg)
{
    DPR_TIMER *dpr_timer = (DPR_TIMER *)arg;

    #ifdef RTAI
       /*rt_printk("RTAI_%s %p rtime=%lld\n", __FUNCTION__, dpr_timer, rt_get_time());*/
    #else
       static DPR_UINT32 cnt = 0;
       if ( !(cnt % 100) ) {
           DPRLIBLOGMSG("ptimer=%p cnt=%d\n", dpr_timer, cnt);
       }
       cnt++;
    #endif
    dpr_timer->fct(dpr_timer->data);
}

/* forward declarations */
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
extern void cp16xx_register_ioctl32_conversion(void);
extern void cp16xx_unregister_ioctl32_conversion(void);
#else
extern long cp16xx_ioctl_compat(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#endif

static int cp16xx_os_open(struct inode *inode, struct file *filp);
static int cp16xx_os_release(struct inode *inode, struct file *filp);
static long cp16xx_os_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int cp16xx_os_mmap(struct file *filp, struct vm_area_struct *vma);
static ssize_t cp16xx_os_read(struct file *filp, char *buf, size_t count, loff_t * f_pos);
static ssize_t cp16xx_os_write(struct file *filp, const char *buf, size_t count, loff_t * f_pos);


/* user interface(OS dependent)

   for Linux based on file operations structure

   cp16xx_os_open - binds sys_open with cp16xx_base_open

   cp16xx_os_release - binds sys_close with cp16xx_base_release

   cp16xx_os_ioctl - checks the memory in request, and dispatch to cp6xx_base_ioctl

   cp16xx_os_mmap_* - provide mmap interface

   cp16xx_os_read_complete - callback, unblocks cp16xx_os_read and transfers data block to user space

   cp16xx_os_read_abort - callback, unblocks cp16xx_os_read, but don't transfer any block

   cp16xx_os_read - binds sys_read with cp16xx_base_read, in success case blocks and waits
                    for a data block(data channels) or incoming event(control channels)

   cp16xx_os_write - binds sys_write with cp16xx_base_write
*/
struct file_operations cp16xx_os_ops = {
    .owner = THIS_MODULE,
    .open    = cp16xx_os_open,
    .release = cp16xx_os_release,
    .mmap    = cp16xx_os_mmap,
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
    .compat_ioctl   = cp16xx_ioctl_compat,
#endif
#endif
#ifdef HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl   = cp16xx_os_ioctl,
#else
    .ioctl   = cp16xx_os_ioctl,
#endif
    .read    = cp16xx_os_read,
    .write   = cp16xx_os_write,
};

static int cp16xx_os_open(struct inode *inode, struct file *filp)
{
    int ret = 0, i;
    struct cp16xx_card_data *card = NULL;

    DPRLIBLOGMSG("begin minor %d, pid %d\n", MINOR(inode->i_rdev), current->pid);

    if(filp->private_data) {
        DPRLIBERRMSG("minor %d, pid %d, private pointer must be NULL\n",
            MINOR(inode->i_rdev), current->pid);
        ret = -ENODEV;
    } else {
        for(i = 0; i < MAX_CP16XX_DEVICES; i++) {
            card = cp16xx_card_ref_get(i);
            if(card && card->os_info.major == MAJOR(inode->i_rdev)) {
                break;
            }
            card = NULL;
        }

        if(card)
            filp->private_data = cp16xx_base_open(card, (MINOR(inode->i_rdev) & 0x0F));
        else
            ret = -ENODEV;
    }

    DPRLIBLOGMSG("end\n");

    return ret;
}

static int cp16xx_os_release(struct inode *inode, struct file *filp)
{
    struct cp16xx_interface *interface;

    DPRLIBLOGMSG("begin minor %d, pid %d\n", MINOR(inode->i_rdev), current->pid);

    interface = (struct cp16xx_interface *)filp->private_data;
    cp16xx_base_release(interface);

    filp->private_data = NULL;

    DPRLIBLOGMSG("end\n");

    return 0;
}

static long cp16xx_os_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct cp16xx_interface *interface;
    char *tmp = NULL;
    int ret = 0;

    interface = (struct cp16xx_interface *)filp->private_data;

    DPRLIBLOGMSG("begin channel=%d, ioctl=%d ifc=%#x \n", interface->channel_number, _IOC_NR(cmd), (unsigned int)interface );

    if (!interface->card)
    	return -EFAULT;

    switch(cmd) {
    case CP16XX_IOCWIRQ:
        {
            //Linux specific implementation, we block
            DPR_WAIT_POINT wait;

            DPR_PREPARE_WAIT_ON_POINT(wait);
            DPR_WAIT_ON_POINT(interface->card->rt_irq, wait);
            DPR_TASK_SHEDULE();
            if (DPR_MUTEX_LOCK(interface->card->os_info.ioctl_bkl))
               return -EFAULT;
            DPR_REMOVE_WAIT_POINT(interface->card->rt_irq, wait);

            ret = 0;
        }
        break;
    default:
        if (DPR_MUTEX_LOCK(interface->card->os_info.ioctl_bkl))
            return -EFAULT;
        if((_IOC_DIR(cmd) & _IOC_WRITE) || (_IOC_DIR(cmd) & _IOC_READ)) {
            tmp = DPR_VMALLOC(_IOC_SIZE(cmd));
            DPRLIBLOGMSG("arg %p, size %d\n", tmp, _IOC_SIZE(cmd));
            if(_IOC_DIR(cmd) & _IOC_WRITE) {
                if(DPR_MEMCPY_FROM_USER(tmp, (void *)arg, _IOC_SIZE(cmd))) {
                    DPR_VFREE(tmp);
                    ret = -EFAULT;
                    goto cp16xx_os_ioctl_fail;
                }
            }
        }
        ret = cp16xx_base_ioctl(interface, cmd, tmp, _IOC_SIZE(cmd), _IOC_SIZE(cmd));
        //DPRLIBLOGMSG("ret %d\n", ret);
        if(ret > 0 && (_IOC_DIR(cmd) & _IOC_READ)) {
            DPR_ASSERT(ret <= _IOC_SIZE(cmd));
            if(DPR_MEMCPY_TO_USER((void *)arg, tmp, ret)) {
                ret = -EFAULT;
            }
        }
        break;
    }

cp16xx_os_ioctl_fail:
    if(tmp)
        DPR_VFREE(tmp);

    DPR_MUTEX_UNLOCK(interface->card->os_info.ioctl_bkl);

    DPRLIBLOGMSG("end: ret=%d \n", ret);

    if(ret >= 0)
        return 0;

    switch(ret) {
    case E_CP16XX_WRONGUSERID:
    case E_CP16XX_WRONGARG:
        return -EINVAL;
    case E_CP16XX_WRONGIOCTL:
        return -ENOTTY;
    case E_CP16XX_MAXREACHED:
        return -EMFILE;
    case E_CP16XX_NOPERMISSION:
        return -EPERM;
    case E_CP16XX_BUSY:
        return -EAGAIN;
    case E_CP16XX_FAULT:
        return -EFAULT;
    case E_CP16XX_NOMEMORY:
        return -ENOMEM;
    case E_CP16XX_IOFAULT:
        return -EFAULT;
    default:
        return ret;
    }
}

static void cp16xx_os_mmap_dma_vm_open(struct vm_area_struct *vma)
{
    struct cp16xx_dma_region *region = (struct cp16xx_dma_region *)vma->vm_private_data;

    DPR_ATOMIC_INC(region->count);
}

static void cp16xx_os_mmap_dma_vm_close(struct vm_area_struct *vma)
{
    struct cp16xx_dma_region *region = (struct cp16xx_dma_region *)vma->vm_private_data;
    void *address = region->virt;
    unsigned long vsize = region->size;
    struct page *page;
    void *page_ptr;
    unsigned long m;

    DPR_ATOMIC_DEC(region->count);
    if(0 == DPR_ATOMIC_READ(region->count)) {
        for(m = 0; m < vsize; m += PAGE_SIZE) {
            page_ptr = (unsigned char *) address + m;
            page = virt_to_page(page_ptr);
            ClearPageReserved(page);
        }
    }
}

static struct vm_operations_struct cp16xx_os_mmap_dma_vm_ops = {
    .open = cp16xx_os_mmap_dma_vm_open,
    .close = cp16xx_os_mmap_dma_vm_close,
};

static void cp16xx_os_mmap_io_vm_open(struct vm_area_struct *vma)
{
    struct cp16xx_pci_bar *bar = (struct cp16xx_pci_bar *)vma->vm_private_data;

    DPR_ATOMIC_INC(bar->count);
}

static void cp16xx_os_mmap_io_vm_close(struct vm_area_struct *vma)
{
    struct cp16xx_pci_bar *bar = (struct cp16xx_pci_bar *)vma->vm_private_data;
    DPR_ATOMIC_DEC(bar->count);
}

static struct vm_operations_struct cp16xx_os_mmap_io_vm_ops = {
    .open = cp16xx_os_mmap_io_vm_open,
    .close = cp16xx_os_mmap_io_vm_close,
};

static int cp16xx_os_mmap_dma_remap(struct vm_area_struct *vma, void *address,
    dma_addr_t physical, unsigned long vsize)
{
    int ret;
    struct page *page;
    void *page_ptr;
    unsigned long m;

    DPRLIBLOGMSG("begin\n");

    for(m = 0; m < vsize; m += PAGE_SIZE) {
        page_ptr = (unsigned char *) address + m;
        page = virt_to_page(page_ptr);
        SetPageReserved(page);
    }

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9)
    ret = remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT, vsize,
        vma->vm_page_prot);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
    ret = remap_page_range(vma, vma->vm_start, physical, vsize, vma->vm_page_prot);
#else
    ret = remap_page_range(vma->vm_start, physical, vsize, vma->vm_page_prot);
#endif
    if(ret) {
        DPRLIBERRMSG("error in remap_[page|pfn]_range %i\n", ret);
        return -EAGAIN;
    }

    DPRLIBLOGMSG("end, virtual 0x%lx physical 0x%lx size 0x%lx\n",
        vma->vm_start, DPR_PHYSICAL_ADDRESS_TO_ULONG(physical), vsize);

    return 0;
}

static int cp16xx_os_mmap_io_remap(struct vm_area_struct *vma,
    unsigned long physical, unsigned long vsize)
{
    int ret;

    DPRLIBLOGMSG("begin\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12)
    ret = io_remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT, vsize, vma->vm_page_prot);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#ifdef __sparc__
    ret = io_remap_page_range(vma, vma->vm_start, physical, vsize, vma->vm_page_prot, 0);
#else
    ret = io_remap_page_range(vma, vma->vm_start, physical, vsize, vma->vm_page_prot);
#endif
#else
#ifdef __sparc__
    ret = io_remap_page_range(vma->vm_start, physical, vsize, vma->vm_page_prot, 0);
#else
    ret = io_remap_page_range(vma->vm_start, physical, vsize, vma->vm_page_prot);
#endif
#endif
#endif
    if(ret) {
        DPRLIBERRMSG("error in io_remap_page_range %i\n", ret);
        return -EAGAIN;
    }

    DPRLIBLOGMSG("end, virtual %lx physical %lx size %lx\n",
        vma->vm_start, physical, vsize);

    return 0;
}

static int cp16xx_os_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct cp16xx_interface *interface;
    unsigned long off, vsize;
    int ret;
    struct cp16xx_dma_region *region = NULL;
    struct cp16xx_pci_bar *bar = NULL;

    DPRLIBLOGMSG("begin\n");

    interface = (struct cp16xx_interface *)filp->private_data;

    if(!interface->app)
        return -EFAULT;

    off = (vma->vm_pgoff << PAGE_SHIFT);
    vsize = vma->vm_end - vma->vm_start;

    if(off >= MMAP_OFFSET_L2_DMA) {
        off -= MMAP_OFFSET_L2_DMA;
        region = &interface->card->l2;
        goto cp16xx_os_mmap_dma;
    } else if(off >= MMAP_OFFSET_DMA) {
        off -= MMAP_OFFSET_DMA;
        region = &interface->card->irt;
        goto cp16xx_os_mmap_dma;
    } else if(off >= MMAP_OFFSET_DPRAM) {
        off -= MMAP_OFFSET_DPRAM;
        bar = &interface->card->bars[PCI_BAR_DPRAM];
    } else if(off >= MMAP_OFFSET_EMIF_CS0) {
        off -= MMAP_OFFSET_EMIF_CS0;
        bar = &interface->card->bars[PCI_BAR_EMIF_CS0];
    } else if(off >= MMAP_OFFSET_IRTE) {
        off -= MMAP_OFFSET_IRTE;
        bar = &interface->card->bars[PCI_BAR_IRTE];
    } else if(off >= MMAP_OFFSET_RAM) {
        off -= MMAP_OFFSET_RAM;
        bar = &interface->card->bars[PCI_BAR_RAM];
    } else if(off >= MMAP_OFFSET_BOOTROM) {
        off -= MMAP_OFFSET_BOOTROM;
        bar = &interface->card->bars[PCI_BAR_BOOTROM];
    } else {
        return -EINVAL;
    }
    if(!bar || off >= bar->bar_size)
        return -EINVAL;
    if((bar->bar_size - off) < vsize)
        return -EFBIG;

    vma->vm_flags |= (VM_IO | VM_RESERVED);
    if(!(ret = cp16xx_os_mmap_io_remap(vma, bar->bar_base_physaddr + off, vsize))) {
        DPR_ATOMIC_INC(bar->count);
        vma->vm_ops = &cp16xx_os_mmap_io_vm_ops;
        vma->vm_private_data = bar;
    }
    goto cp16xx_os_mmap_quit;

cp16xx_os_mmap_dma:
    if(!region || !region->virt || off >= region->size)
        return -EINVAL;
    if((region->size - off) < vsize)
        return -EFBIG;

    if(!(ret = cp16xx_os_mmap_dma_remap(vma, region->virt + off, region->phys, vsize))) {
        DPR_ATOMIC_INC(region->count);
        vma->vm_ops = &cp16xx_os_mmap_dma_vm_ops;
        vma->vm_private_data = region;
    }

cp16xx_os_mmap_quit:
    DPRLIBLOGMSG("end\n");

    return ret;
}

void cp16xx_os_read_complete(struct cp16xx_channel *channel)
{
    DPRLIB_TRC(" User WAKE-UP ch=%d \n", channel->channel_number);
    DPR_CHANNEL_WAKEUP(channel);
}

void cp16xx_os_read_abort(struct cp16xx_channel *channel)
{
    DPR_CHANNEL_WAKEUP(channel);
}

static ssize_t cp16xx_os_read(struct file *filp, char *buf, size_t count, loff_t * f_pos)
{
    struct cp16xx_interface *interface;
    struct cp16xx_block *block;
    int ret;

    DPRLIBLOGMSG("begin\n");

    interface = (struct cp16xx_interface *)filp->private_data;

    ret = cp16xx_base_read(interface, buf, count);

    if(ret < 0)
        return -ENOENT;

    if(interface->channel_number == CONTROL_INTERFACE) {
        ret = cp16xx_os_syncronize_irt(interface, count);
    } else {
        DPRLIB_TRC(" >WAIT-FOR-WAKEUP ch=%d \n", interface->channel->channel_number);
        if(DPR_CHANNEL_WAIT_FOR_WAKEUP(interface->channel)) {
            DPRLIBLOGMSG("got a signal by waiting on wakeup\n");
            return -EAGAIN;
        }

        DPRLIB_TRC(" < WOKE-UP ch=%d \n", interface->channel->channel_number);
        block = BLOCK_remove_head(interface->channel);

        if(!block) {
            DPRLIBLOGMSG("channel %u, queue is empty\n", interface->channel_number);
            return -EAGAIN;
        }

        DPRLIBLOGMSG("channel %u msg_length %lu, user_length %Zu\n",
                interface->channel_number, block->length, count);

        if(count < block->length) {
            ret = block->length;
        } else {
            if(!DPR_IS_VALID_KMEM(buf)) {
                ret = DPR_MEMCPY_TO_USER(buf, block->addr, block->length);
                if(ret >= 0) {
                    ret = block->length - ret;
                }
            } else {
                memcpy(buf, block->addr, block->length);
                ret = block->length;
            }
        }
        BLOCK_free(block);
    }

    DPRLIBLOGMSG("end\n");

    return ret;
}

ssize_t cp16xx_os_write(struct file *filp, const char *buf, size_t count, loff_t * f_pos)
{
    struct cp16xx_interface *interface;
    int ret = 0;
    void *tmp = NULL;

    DPRLIBLOGMSG("begin\n");

    interface = (struct cp16xx_interface *)filp->private_data;

    if(interface->channel_number == CONTROL_INTERFACE && count == CP16XX_OPDONE)
        ret = cp16xx_base_write(interface, buf, count);
    else if(!DPR_IS_VALID_KMEM(buf)) {
        tmp = DPR_VMALLOC(count);
        if(!tmp)
            return -ENOMEM;
        if(!DPR_MEMCPY_FROM_USER(tmp, buf, count)) {
            ret = cp16xx_base_write(interface, tmp, count);
        } else {
            ret = -EFAULT;
        }
        DPR_VFREE(tmp);
    } else {
        ret = cp16xx_base_write(interface, buf, count);
    }

    DPRLIBLOGMSG("end\n");
    return ret;
}

/* helper proides OS depedant information */
int cp16xx_os_get_info(struct cp16xx_card_data *card, char *buf, int buflen)
{
    return DPR_SNPRINTF(buf, buflen, "pci %s, major %d, irq %d",
        pci_name(card->os_info.pdev), card->os_info.major, card->os_info.irq);
}

/* resources related functions(OS dependent)

   cp16xx_os_pci_init_resources - requests PCI, DMA, IRQ resources and start cp16xx_daemon

   cp16xx_os_pci_uninit_resources - stops cp16xx_daemon, and releases all resources */
static int cp16xx_os_pci_init_resources(struct cp16xx_card_data *card)
{
    int err, bar;
    unsigned long bar_start, bar_end, bar_flag;

    if((err = pci_set_dma_mask(card->os_info.pdev, DMA_VALID_HOST_MASK)) ||
        (err = pci_set_consistent_dma_mask(card->os_info.pdev, DMA_VALID_HOST_MASK)))
        return err;

    /* accessing the configuration space is vital to the driver because
       it is the only way it can find out where the device is mapped in memory and in the I/O space. */
    if((err = pci_request_regions(card->os_info.pdev, "cp16xx")))
        return err;

    for(bar = PCI_BAR_FIRST; bar < PCI_BAR_MAX; ++bar) {

        /* the I/O regions of PCI devices have been integrated in the generic resource management
           The function returns the first address (memory address or I/O port number)
           PCI I/O regions associated with the current bar */
        bar_start = pci_resource_start(card->os_info.pdev, bar);
        bar_end = pci_resource_end(card->os_info.pdev, bar);
        bar_flag = pci_resource_flags(card->os_info.pdev, bar);

        if(bar_start) {
            DPRLIBLOGMSG("bar%i start=0x%lx size=0x%lx \n",
                bar, bar_start, (bar_end - bar_start + 1));

            card->bars[bar].bar_base_physaddr = bar_start;
            /* don't waste address space */
            switch(bar) {
            case PCI_BAR_IRTE:
                card->bars[bar].bar_size = MAX_SIZE_IRTE;
                break;
            case PCI_BAR_EMIF_CS0:
                card->bars[bar].bar_size = MAX_SIZE_EMIF_CS0;
                break;
            case PCI_BAR_DPRAM:
                card->bars[bar].bar_size = MAX_SIZE_DPRAM;
                break;
            default:
                card->bars[bar].bar_size = bar_end - bar_start + 1;
            }
            DPR_ATOMIC_SET(card->bars[bar].count, 0);
        }
    }

    err = cp16xx_pci_init(card);
    if(err)
        goto cp16xx_os_pci_init_fail;

    err = cp16xx_dma_init(card);
    if(err)
        goto cp16xx_os_dma_init_fail;

    err = cp16xx_os_irq_init(card);
    if(err)
        goto cp16xx_os_irq_init_fail;

    DPRLIBLOGMSG("create new kernel thread\n");
    card->daemon_handle = kernel_thread(cp16xx_daemon,
        card, CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
    cp16xx_daemon_work(card, DPRLIB_START);


    DPRLIBLOGMSG("end\n");

    return 0;

cp16xx_os_irq_init_fail:
    /* init routine cares about error handling */

cp16xx_os_dma_init_fail:
    cp16xx_dma_uninit(card);

cp16xx_os_pci_init_fail:
    cp16xx_pci_uninit(card);

    return err;
}

static int cp16xx_os_pci_uninit_resources(struct cp16xx_card_data *card)
{
    DPRLIBLOGMSG("begin\n");

    DPR_WAKEUP_QUEUE(card->rt_irq);
    /* TODO propably kill outstanding applications? */
    cp16xx_daemon_work(card, DPRLIB_STOP);

    while(card->daemon_handle)
        DPR_TASK_DELAY(100);


    cp16xx_os_irq_uninit(card);

    cp16xx_dma_uninit(card);

    cp16xx_pci_uninit(card);

    DPRLIBLOGMSG("end\n");

    return 0;
}

/* instances related functions(OS dependent)

   cp16xx_os_pci_probe - create instances of CP16XX, registers it by OS

   cp16xx_os_pci_remove - destroy instance of CP16XX */
static int __devinit cp16xx_os_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int err = 0;
    struct cp16xx_card_data *card;

    DPRLIBLOGMSG("begin\n");

    /* Enable device in PCI config */
    if(pci_enable_device(pdev)) {
        DPRLIBERRMSG("cannot enable PCI device\n");
        return -ENXIO;
    }

    pci_set_master(pdev);

    /* alloc private part of data */
    card = DPR_ZALLOC(sizeof (struct cp16xx_card_data));
    if(!card) {
        DPRLIBERRMSG("alloc cp16xx_card_data failed\n");
        err = -ENOMEM;
        goto dpr_zalloc_fail;
    }

    err = cp16xx_card_init(card);
    if(err) {
        DPRLIBERRMSG("unable to init card structure, error %i\n", err);
        goto cp16xx_card_init_failed;
    }

    DPR_MUTEX_CREATE_UNLOCKED(card->os_info.ioctl_bkl);

    err = register_chrdev(major, "cp16xx", &cp16xx_os_ops);
    if(err && !major) {
        /* dynamic major */
        DPRLIBLOGMSG("got major %d allocated\n", err);
    } else if(err) {
        DPRLIBERRMSG("unable to register major %d, error %i\n", major, err);
        goto register_chrdev_fail;
    }

    card->os_info.major = err ? err : major;
    card->os_info.irq = pdev->irq;
    card->os_info.pdev = pdev;

    pci_set_drvdata(pdev, card);

    err = cp16xx_os_pci_init_resources(card);
    if(err)
        goto cp16xx_os_pci_init_failed;

#ifdef NETWORK_DRIVER
    cp16xx_net_card_init(card);
#endif
    DPRLIBLOGMSG("end\n");

    return 0;

cp16xx_os_pci_init_failed:
    pci_release_regions(pdev);
    pci_set_drvdata(pdev, NULL);
    unregister_chrdev(card->os_info.major, "cp16xx");
    card->os_info.pdev = NULL;

register_chrdev_fail:
    cp16xx_card_uninit(card);

cp16xx_card_init_failed:
    DPR_FREE(card);

dpr_zalloc_fail:
    pci_disable_device(pdev);

    return err;
}

static void __devexit cp16xx_os_pci_remove(struct pci_dev *pdev)
{
    struct cp16xx_card_data *card;

    DPRLIBLOGMSG("begin\n");

    card = (struct cp16xx_card_data *)pci_get_drvdata(pdev);
    if(!card)
        return;

#ifdef NETWORK_DRIVER
    cp16xx_net_card_uninit(card);
#endif

    DPR_MUTEX_DESTROY(card->ioctl_bkl);
    cp16xx_os_pci_uninit_resources(card);

    pci_release_regions(pdev);
    pci_set_drvdata(pdev, NULL);
    unregister_chrdev(card->os_info.major, "cp16xx");
    card->os_info.pdev = NULL;

    cp16xx_card_uninit(card);

    DPR_FREE(card);

    pci_disable_device(pdev);

    DPRLIBLOGMSG("end\n");
}

/* PCI and devices ID table for CP16XX */
static struct pci_device_id cp16xx_pci_id_table[] = {
    {PCI_VENDOR_SIEMENS_AG, PCI_ID_PROTO, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {PCI_VENDOR_SIEMENS_AG, PCI_ID_CP1616_1, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {PCI_VENDOR_SIEMENS_AG, PCI_ID_CP1616_2, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {PCI_VENDOR_SIEMENS_AG, PCI_ID_CP1604_1, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {0,0,0,0,0,0,0}
};

/* main driver operations structure */
static struct pci_driver cp16xx_driver_ops = {
    .name = "cp16xx",
    .id_table = cp16xx_pci_id_table,
    .probe = cp16xx_os_pci_probe,
    .remove = __devexit_p(cp16xx_os_pci_remove),
};

/* entry function by loading of driver */
int __init cp16xx_os_driver_init(void)
{
    DPRLIBINFMSG("load SIEMENS CP1604, CP1616 driver. Version %s\n",
        cp16xx_driver_version);

    cp16xx_init_global_data();
#ifdef NETWORK_DRIVER
    cp16xx_net_init_global_data();
#endif
#ifndef MODULE
    #error implement option handling if you need it in non-modular build
#endif

#ifdef CONFIG_PCI
    DPRLIBLOGMSG("calls pci_module_init()\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    if(pci_module_init(&cp16xx_driver_ops))
#else
    if(pci_register_driver(&cp16xx_driver_ops))
#endif
        return -ENODEV;
#else
    return -ENODEV;
#endif
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
    cp16xx_register_ioctl32_conversion();
#endif
#endif
#ifdef RTAI
    rt_set_oneshot_mode();
    start_rt_timer(0);
#endif
    DPRLIBLOGMSG("end\n");
    return 0;
}

/* entry function by unloading of driver */
void __exit cp16xx_os_driver_cleanup(void)
{
    DPRLIBLOGMSG("begin\n");
#ifdef RTAI
    stop_rt_timer();
#endif
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
    cp16xx_unregister_ioctl32_conversion();
#endif
#endif
    pci_unregister_driver(&cp16xx_driver_ops);

#ifdef NETWORK_DRIVER
    cp16xx_net_uninit_global_data();
#endif
    cp16xx_uninit_global_data();

    DPRLIBLOGMSG("end\n");
}

void cp16xx_syslog(char *file, int line, char *msg)
{
  printk("cp16xx %s file <%s> line <%d>", msg, file, line);
}




/* driver entry and exit points for linux
   to register the names of our initialization and exit routines */
module_init(cp16xx_os_driver_init);
#ifdef MODULE
module_exit(cp16xx_os_driver_cleanup);
#endif

