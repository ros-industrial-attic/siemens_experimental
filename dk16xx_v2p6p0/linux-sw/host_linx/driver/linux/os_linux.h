/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e               os_linux.h                                         */
/*****************************************************************************/
/*  D e s c r i p t i o n:  Common header declarations                       */
/*                          This is for Linux !!!                            */
/*****************************************************************************/
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

#ifndef _OS_LINUX_H
#define _OS_LINUX_H

/* we have split between kernel and user space */
#if defined(__KERNEL__)
    #include <linux/version.h>
    #include <linux/module.h>
    #include <linux/compiler.h>
    #include <linux/kernel.h>
    #include <linux/fs.h>
    #include <linux/pci.h>
    #include <linux/interrupt.h>
    #include <linux/vmalloc.h>
    #include <linux/sched.h>

    #include <linux/types.h>
    #include <linux/spinlock.h>
    #include <linux/sem.h>
    #include <linux/wait.h>
    #include <linux/delay.h>
    #include <linux/time.h>      /* PL: added do_gettimeofdate() */
    #include <linux/jiffies.h>   /* PL: time ticks  */

    #include <asm/page.h>
    #include <asm/uaccess.h>
    #include <asm/io.h>

    #if defined(RTAI)
        #include "rtai.h"
        #include "rtai_sem.h"
        #include "rtai_sched.h"
        #include "rtai_tasklets.h"
    #elif defined(XENOMAI)
        #include "native/sem.h"
        #include "native/intr.h"
        #include "native/nrti.h"
    #endif

    /* CP16xx can address only DMA memory under 512 MB,
       but Linux on UltraSPARC does support only full 32 bit DMA mask.
       On this architecture it can happens, that you cannot do L2 and/or PNIO IRT */
    #ifdef __sparc__
        #define DMA_VALID_HOST_MASK     0x00000000FFFFFFFFULL
    #else
        #define DMA_VALID_HOST_MASK     0x000000001FFFFFFFULL
    #endif

    /*****************************************************************************/
    /* SWAPPING of the DPRAM-Structure when Host or Firmware are not little      */
    /* endian. Use native API of Linux kernel                                    */
    /*****************************************************************************/

    #define CPU_TO_LE16(c) cpu_to_le16(c)
    #define LE_TO_CPU16(c) le16_to_cpu(c)

    #define CPU_TO_LE(c) cpu_to_le32(c)
    #define LE_TO_CPU(c) le32_to_cpu(c)

    #define CPU_TO_BE16(c) cpu_to_be16(c)
    #define BE_TO_CPU16(c) be16_to_cpu(c)

    #define CPU_TO_BE(c) cpu_to_be32(c)
    #define BE_TO_CPU(c) be32_to_cpu(c)

    /* elementar types */
    typedef u8      DPR_CHAR;
    typedef u16     DPR_UINT16;
    typedef void    DPR_VOID;
    typedef u32     DPR_UINT32;
    typedef volatile u32  DPR_VUINT32;
    typedef s16 DPR_INT16;
    typedef s32 DPR_INT32;

    /* Linux does provide these ANSI types in <linux/types.h>
    typedef DPR_CHAR    uint8_t;
    typedef DPR_UINT16  uint16_t;
    typedef DPR_UINT32  uint32_t;
    */

    /* access PCI memory only with these macros */
    #define DPR_READ_UINT32(addr) readl(addr)
    #define DPR_WRITE_UINT32(val, addr) writel((val), (addr))
    #define DPR_READ_UINT8(addr) readb(addr)
    #define DPR_WRITE_UINT8(val, addr) writeb((val), (addr))

    /* types of PCI and DMA resources */
    #ifdef __sparc__
        typedef unsigned long DPR_PCI_PHYSICAL_ADDRESS;
    #else
        typedef dma_addr_t DPR_PCI_PHYSICAL_ADDRESS;
    #endif
    typedef dma_addr_t DPR_DMA_PHYSICAL_ADDRESS;

    /* conversion macros */
    #define DPR_PTR_TO_ULONG(x)                ((unsigned long)(x))
    #define DPR_PTR_TO_INT(x)                  ((int)(x))
    #define DPR_ULONG_TO_PTR(x)                ((unsigned char *)(x))
    #define DPR_PHYSICAL_ADDRESS_TO_ULONG(x)   ((unsigned long)(x))
    #define DPR_PHYSICAL_ADDRESS_TO_PTR(x)     ((unsigned char *)DPR_PHYSICAL_ADDRESS_TO_ULONG(x))
    #define DPR_VIRTUAL_ADDRESS_TO_PTR(x)      ((unsigned char *)(x))
    #define DPR_PHYSICAL_ADDRESS(x)            (x)

    /* helper macros */
    #define DPR_STRNCPY strncpy
    #define DPR_SNPRINTF snprintf
    #define DPR_STRLEN(str, strmaxlen, retlen) (retlen = strlen(str))
    #define DPR_IS_ADMIN (capable(CAP_SYS_ADMIN))

    /* file related kernel <-> kernel L2 API */
    static inline struct file *filp_open_checked(const char* name)
    {
        struct file * hfile;
        int err;

        hfile = filp_open(name, 0, 0);
        if(IS_ERR(hfile)) {
            err = PTR_ERR(hfile);
            return NULL;
        }
        return hfile;
    };

    typedef struct file * DPR_DRV_HANDLE;
    #define DPR_DRV_OPEN(name) filp_open_checked(name)
    #define DPR_DRV_CLOSE(name) filp_close(name, NULL)
    #define DPR_DRV_WRITE(file, buf, len) (file->f_op->write(file, buf, len))
    #define DPR_DRV_READ(file, buf, len) (file->f_op->read(file, buf, len))
    #define DPR_DRV_ERROR(file) (0)
    #define DPR_DRV_IOCTL(file, cmd, arg, size_in, siz_out) (file->f_op->ioctl(file->f_dentry->d_inode, file, cmd, (int)arg))
    #define DPR_DRV_MMAP(file, addr, len, usewr_id) (bus_to_virt(addr))
    #define DPR_DRV_MUNMAP(file, ptr, len, user_id) (0)
    #define DPR_STRERROR() ("")

    /* spinlock API */
    typedef spinlock_t DPR_SPINLOCK;
    typedef unsigned long DPR_SPINLOCK_FLAGS;
    #define DPR_SPINLOCK_INIT(a)      spin_lock_init(&a)
    #define DPR_SPINLOCK_UNINIT(a)
    #define DPR_SPINLOCK_LOCK(a, b)   spin_lock_irqsave(&a, b)
    #define DPR_SPINLOCK_UNLOCK(a, b) spin_unlock_irqrestore(&a, b)

#ifdef NETWORK_DRIVER
    typedef DPR_SPINLOCK L2_KERNEL_SPINLOCK;
    typedef DPR_SPINLOCK_FLAGS L2_KERNEL_SPINLOCK_FLAGS;

    #define L2_KERNEL_SPINLOCK_INIT(a) DPR_SPINLOCK_INIT(a)
    L2_KERNEL_SPINLOCK *l2_read_spinlock(DPR_UINT32 Handle);
    L2_KERNEL_SPINLOCK *l2_write_spinlock(DPR_UINT32 Handle);

    #define L2_KERNEL_RSPINLOCK_LOCK(a, b) spin_lock_irqsave(l2_read_spinlock(a), b)
    #define L2_KERNEL_RSPINLOCK_UNLOCK(a, b) spin_unlock_irqrestore(l2_read_spinlock(a), b)
    #define L2_KERNEL_WSPINLOCK_LOCK(a, b) spin_lock_irqsave(l2_write_spinlock(a), b)
    #define L2_KERNEL_WSPINLOCK_UNLOCK(a, b) spin_unlock_irqrestore(l2_write_spinlock(a), b)

    #define L2_KERNEL_SPINLOCK_LOCK_IRQ_OFF(a) spin_lock(&a)
    #define L2_KERNEL_SPINLOCK_UNLOCK_IRQ_ON(a) spin_unlock(&a)
#endif

    /* atomic API */
    typedef atomic_t   DPR_ATOMIC;
    #define DPR_ATOMIC_INC(x)  atomic_inc(&(x))
    #define DPR_ATOMIC_DEC(x)  atomic_dec(&(x))
    #define DPR_ATOMIC_READ(x) atomic_read(&(x))
    #define DPR_ATOMIC_SET(x, y) atomic_set(&(x), y)

    #define DPR_IOREMAP_NOCACHE(_card, _physaddr, _size) ioremap_nocache(DPR_PHYSICAL_ADDRESS_TO_ULONG(_physaddr), (_size))
    #define DPR_IOUNMAP(_card, _ptr, _size)            iounmap((_ptr))

    /* DMA API */
    #define DPR_GET_PAGE_SIZE PAGE_SIZE
    #define DPR_ALLOC_CONSISTENT_DMA(card, size, phys) (pci_alloc_consistent(card->os_info.pdev, size, phys))
    #define DPR_FREE_CONSISTENT_DMA(card, size, virt, phys) (pci_free_consistent(card->os_info.pdev, size, virt, phys))

    /* time API */
    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,6)
        static inline unsigned long local_msecs_to_jiffies(const unsigned int m) {
        if (HZ <= 1000 && !(1000 % HZ))
            return (m + (1000 / HZ) - 1) / (1000 / HZ);
        else if (HZ > 1000 && !(HZ % 1000))
            return m * (HZ / 1000);
        else
            return (m * HZ + 999) / 1000;
        }
    #else
        #define local_msecs_to_jiffies msecs_to_jiffies
    #endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,6) */

    #define DPR_TASK_DELAY(msecs) (schedule_timeout(local_msecs_to_jiffies(msecs)))
    #define DPR_TASK_DELAY_UNINTERRUPTIBLE(msecs) \
        { set_current_state(TASK_UNINTERRUPTIBLE); schedule_timeout(local_msecs_to_jiffies(msecs)); }
    #define DPR_TASK_SHEDULE() { schedule(); }

    void DPR_LINUX_TIMER_FCT(unsigned long arg);

#if defined(RTAI)
        typedef struct {
            struct rt_tasklet_struct timer;
            void (*fct)(void *arg);
            void *data; /* app_ref */
        } DPR_TIMER;

        #define DPR_TIMER_INIT(x, y, z) {\
            (x)->fct = (y);\
            (x)->data = (void *)(z);\
            /*printk("DPR_TIMER_INIT %p\n", x);*/\
        }
        #define DPR_TIMER_START(x, t /*millisec*/) {\
            int ret;\
            ret = rt_insert_timer( &(x)->timer, 0 /*prio*/, \
                              rt_get_time() + nano2count(t*1000000LL) /*RTIME firing_time*/,\
                              0 /*RTIME period=0 oneshut timer*/, DPR_LINUX_TIMER_FCT,\
                               (unsigned long)(x) /*fct arg*/, 0 /*pid=kernel*/);\
            /*printk("DPR_TIMER_START %p t=%d rt=%lld ret=%i\n", x, t, rt_get_time(), ret);*/\
        }
        #define DPR_TIMER_DELETE_SYNC(x) {\
            /*printk("DPR_TIMER_DELETE_SYNC %p\n", x);*/\
            /*oneshot timer is not active after expiration, rt_remove_timer (&(x)->timer);*/\
        }
#else /* defined(RTAI) */

    typedef struct {
        int init;
        struct timer_list tl;
        void (*fct)(void *arg);
        void *data;
    } DPR_TIMER;

    #define DPR_TIMER_INIT(x, y, z) {\
        (x)->init = 1;\
        init_timer(&(x)->tl);\
        (x)->fct = (y);\
        (x)->data = (void *)(z);\
        (x)->tl.function = DPR_LINUX_TIMER_FCT;\
        (x)->tl.data = (unsigned long)(x);\
    }
    #define DPR_TIMER_START(x, t) {\
        (x)->tl.expires = jiffies + local_msecs_to_jiffies(t);\
        add_timer(&(x)->tl);\
    }
    #define DPR_TIMER_DELETE_SYNC(x) {\
        del_timer_sync(&(x)->tl);\
    }
#endif /* defined(RTAI) */

    /* semaphore API */
    typedef struct semaphore DPR_SEMAPHORE;
    #define DPR_SEM_CREATE(semObj) (sema_init(&(semObj), 0),0)
    #define DPR_SEM_WAIT(semObj) (down(&(semObj)))
    #define DPR_SEM_WAIT_INTERRUPTIBLE(semObj) (down_interruptible(&(semObj)))

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    static inline int down_timeout(struct semaphore *sema, unsigned long msecs) {
        unsigned long jiffies;
        int ret, iret;

        jiffies = local_msecs_to_jiffies(msecs);
        ret = jiffies;
        {
            DEFINE_WAIT(__wait);

            prepare_to_wait(&sema->wait, &__wait, TASK_INTERRUPTIBLE);
            if (signal_pending(current))
                ret = -1;
            else
                ret = schedule_timeout(ret);

            finish_wait(&sema->wait, &__wait);
        }

        if(!ret && !down_trylock(sema))
            iret = 0;
        else if(!ret)
            iret = 1; /* timeout */
        else
            iret = -1; /* signal interruptes */

        return iret;
    }
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28) */

    #define DPR_SEM_WAIT_TIME(semObj, msecs) (down_timeout(&semObj, msecs))
    #define DPR_SEM_POST(semObj) (up(&(semObj)))
    #define DPR_SEM_DESTROY(semObj)
    #define DPR_SEM_WAIT_FOR_WAKEUP(semObj, msecs) (down_timeout(&semObj, msecs) && (up(&semObj), 1))

    /* wait queue API */
#if defined(RTAI)
    typedef SEM* DPR_WAIT_QUEUE_IRT;
    #define DPR_INIT_WAIT_QUEUE_IRT(x)
    #define DPR_UNINIT_WAIT_QUEUE_IRT(x)
#elif defined(XENOMAI)
    typedef RT_SEM DPR_WAIT_QUEUE_IRT;
    #define DPR_INIT_WAIT_QUEUE_IRT(x)
    #define DPR_UNINIT_WAIT_QUEUE_IRT(x)
#else
    typedef wait_queue_head_t DPR_WAIT_QUEUE_IRT;
    #define DPR_INIT_WAIT_QUEUE_IRT(x) init_waitqueue_head(&(x))
    #define DPR_UNINIT_WAIT_QUEUE_IRT(x)
#endif

    typedef wait_queue_head_t DPR_WAIT_QUEUE_RT;
    #define DPR_INIT_WAIT_QUEUE_RT(x) init_waitqueue_head(&(x))
    #define DPR_UNINIT_WAIT_QUEUE_RT(x)

#if defined(RTAI)
    #define DPR_WAKEUP_QUEUE_IRT(x) rt_sem_broadcast(x)
#elif defined(XENOMAI)
    #define DPR_WAKEUP_QUEUE_IRT(x) rt_sem_broadcast(&x)
#else
    #define DPR_WAKEUP_QUEUE_IRT(x) wake_up(&(x))
#endif

    #define DPR_WAKEUP_QUEUE(x) wake_up(&(x))

    typedef wait_queue_t DPR_WAIT_POINT;
    #define DPR_PREPARE_WAIT_ON_POINT(point) {\
        init_waitqueue_entry(&point, current);\
        current->state = TASK_INTERRUPTIBLE;\
    }
    #define DPR_WAIT_ON_POINT(queue, point) add_wait_queue(&queue, &point)
    #define DPR_REMOVE_WAIT_POINT(queue, point) remove_wait_queue(&queue, &point)

    /* mutex API */
    typedef struct semaphore DPR_MUTEX;
    #define DPR_MUTEX_CREATE_UNLOCKED(muxObj) sema_init(&muxObj, 1)
    #define DPR_MUTEX_LOCK(muxObj) down_interruptible(&muxObj)
    #define DPR_MUTEX_UNLOCK(muxObj) up(&muxObj)
    #define DPR_MUTEX_DESTROY(muxObj)

    /* thread API */
    typedef int DPR_THREAD_HANDLE;
    typedef int DPR_THREAD_RETURN_TYPE;
    #define DPR_THREAD_DECL
    #define DPR_THREAD_END() return 0;
    #define DPR_THREAD_CREATE(ptid, name, func, arg)\
        ((*ptid = kernel_thread(func, arg, CLONE_FS | CLONE_FILES | CLONE_SIGHAND)) > 0)
    #define DPR_THREAD_DAEMONIZE(name) daemonize(name)
    #define DPR_THREAD_ALLOW_SIGNALS allow_signal( SIGTERM)
    #define DPR_DELETE_THREAD(hThread) (0 == kill_proc(hThread, SIGTERM, 1))

    /* logging API */
    /* use native printk interface */

    /* DPRLIB_TRC: this trace macro is used in the release version to trace special problem cases
     *             PL: last usage: analysis of mgt-ring receive interrupts and behavior
     */
    //#define DPRLIB_TRC(fmt, args...) printk(KERN_DEBUG "%lu[ms] %s: " fmt,(jiffies * 1000 / HZ),  __FUNCTION__ , ## args)
    #define DPRLIB_TRC(fmt, args...)
    //#define DPRLIBCHATMSG(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
    #define DPRLIBCHATMSG(fmt, args...)

    #ifdef DEBUG
        #define DPRLIBLOGMSG(fmt, args...) printk(KERN_DEBUG "%lu[ms] %s: " fmt,(jiffies * 1000 / HZ),  __FUNCTION__ , ## args)
        //#define DPRLIBLOGMSG(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
    #else
        //#define DPRLIBLOGMSG(fmt, args...) printk(KERN_DEBUG "%lu[ms] %s: " fmt,(jiffies * 1000 / HZ),  __FUNCTION__ , ## args)
        //#define DPRLIBLOGMSG(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
        #define DPRLIBLOGMSG(fmt, args...)
    #endif /* DEBUG */

    #define DPRLIBINFMSG(fmt, args...) printk(KERN_INFO "%s: " fmt, __FUNCTION__ , ## args)
    #define DPRLIBWARNMSG(fmt, args...) printk(KERN_WARN "%s: " fmt, __FUNCTION__ , ## args)
    #define DPRLIBERRMSG(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)


    /* copy API */
    /* returns count of nottransferred bytes */
    #define DPR_MEMCPY_TO_PCI(to, from, len) (memcpy_toio((to), (from), (len)), 0)
    #define DPR_MEMCPY_FROM_PCI(to, from, len) (memcpy_fromio((to), (from), (len)), 0)
    #define DPR_MEMCPY_TO_USER(to, from, len) (copy_to_user((to), (from), (len)))
    #define DPR_MEMCPY_FROM_USER(to, from, len) (copy_from_user((to), (from), (len)))

    /* memory API */
    #define DPR_IS_VALID_KMEM(x) (((unsigned long)(x)) >= PAGE_OFFSET)
    /*
       DPR_VMALLOC - virtual memory
       DPR_ZALLOC  - kernel memory(not HIGMEM), zeroed  */
    #define DPR_VMALLOC(length) vmalloc(length)
    #define DPR_VFREE(ptr) vfree(ptr)
    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
        static inline void *kcalloc(size_t size, int count, int flags)
        {
            void *p = kmalloc(size * count, flags);
            if(p)
                memset(p, 0, size * count);

            return p;
        }
    #endif
    #define DPR_ZALLOC(size) kcalloc(size, 1, GFP_KERNEL)
    #define DPR_FREE(mObj) kfree(mObj)

    void cp16xx_syslog(char *file, int line, char *msg);

    /* assert API */
    #define DPR_ASSERT(x) {if(!(x)) {cp16xx_syslog(__FILE__, __LINE__, "Assertion failed"); while(1) DPR_TASK_DELAY(500);} }

    /* we can later raise a panic */
    /* #define DPR_ASSERT(x) WARN_ON(!(x)) */

    /* structures extensions and macros */
    struct DPR_CHANNEL_OS_INFO {
        DPR_SEMAPHORE rwakeup;
    };

    #define DPR_CHANNEL_INIT_OS(x)         { memset(&(x)->os_info, 0, sizeof((x)->os_info));\
                                            DPR_SEM_CREATE((x)->os_info.rwakeup); }
    #define DPR_CHANNEL_UNINIT_OS(x)       { DPR_SEM_DESTROY((x)->os_info.rwakeup); }
    #define DPR_CHANNEL_WAIT_FOR_WAKEUP(x) (DPR_SEM_WAIT_INTERRUPTIBLE((x)->os_info.rwakeup) &&\
                                           (DPR_SEM_POST((x)->os_info.rwakeup), 1))
    #define DPR_CHANNEL_WAKEUP(x)          (DPR_SEM_POST((x)->os_info.rwakeup))
    #define DPR_CHANNEL_LOCK(x)            (down(&((x)->wmutex)))
    #define DPR_CHANNEL_UNLOCK(x)          (DPR_MUTEX_UNLOCK((x)->wmutex))

    #define DPR_CHANNEL_RESET_SEMA(x)      { DPR_SEM_DESTROY((x)->os_info.rwakeup); \
                                             DPR_SEM_CREATE((x)->os_info.rwakeup); }

    struct DPR_APP_OS_INFO {
        int unused;
    };

    #define DPR_APP_INIT_OS(x)             { memset(&(x)->os_info, 0, sizeof((x)->os_info)); }
    #define DPR_APP_UNINIT_OS(x)

    struct DPR_CARD_OS_INFO {
        struct pci_dev *pdev;
        int major;
        int irq;
#if defined(RTAI)
        SEM     rt_softirqsem;
        RT_TASK *rt_softirqtask;
#elif defined(XENOMAI)
        RT_INTR rt_intr;
        rt_nrtsig_t rt_tasklet;
#endif
#ifdef NETWORK_DRIVER
        void *net_device;
#endif
    	/* this mutex replaces the legacy big kernel lock in file_operations.ioctl */
    	DPR_MUTEX ioctl_bkl;
    };

    #define DPR_CARD_INIT_OS(x)            { memset(&(x)->os_info, 0, sizeof((x)->os_info)); }
    #define DPR_CARD_UNINIT_OS(x)

    #define atol(ptr) simple_strtol((ptr), 0, 10)

#else /* ifndef __KERNEL__ */
    #include <inttypes.h>
    #include <errno.h>
    #include <assert.h>
    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <stdarg.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <pthread.h>
    #include <signal.h>
    #include <sys/mman.h>
    #include <sys/types.h>
    #include <sys/time.h>
    #include <sys/timeb.h>
    #include <sys/ioctl.h>
    #include <sys/stat.h>
    #include <sys/shm.h>
    #include <semaphore.h>

    /*****************************************************************************/
    /* SWAPPING of the DPRAM-Structure when Host or Firmware are not little      */
    /* endian.                                                                   */
    /*****************************************************************************/
    #define SWAP_16(var)                  \
        (  ((( (var)) & 0xFF00L) >>  8)   \
        + ((( (var)) & 0x00FFL) << 8))

    #define SWAP_32(var)                     \
        (  ((( (var)) & 0xFF000000L) >> 24)  \
        + ((( (var)) & 0x00FF0000L) >>  8)   \
        + ((( (var)) & 0x0000FF00L) <<  8)   \
        + ((( (var)) & 0x000000FFL) << 24))

    #if __BYTE_ORDER == __BIG_ENDIAN
        #define CPU_TO_LE16(c) SWAP_16(c)
        #define LE_TO_CPU16(c) SWAP_16(c)
        #define CPU_TO_LE(c) SWAP_32(c)
        #define LE_TO_CPU(c) SWAP_32(c)

        #define CPU_TO_BE16(c) (c)
        #define BE_TO_CPU16(c) (c)
        #define CPU_TO_BE(c) (c)
        #define BE_TO_CPU(c) (c)
    #else
        #define CPU_TO_LE16(c) (c)
        #define LE_TO_CPU16(c) (c)
        #define CPU_TO_LE(c) (c)
        #define LE_TO_CPU(c) (c)

        #define CPU_TO_BE16(c) SWAP_16(c)
        #define BE_TO_CPU16(c) SWAP_16(c)
        #define CPU_TO_BE(c) SWAP_32(c)
        #define BE_TO_CPU(c) SWAP_32(c)
    #endif

    /* elementar types */
    typedef uint8_t      DPR_CHAR;
    typedef uint16_t     DPR_UINT16;
    typedef void         DPR_VOID;
    typedef uint32_t     DPR_UINT32;
    typedef volatile uint32_t DPR_VUINT32;
    typedef int16_t     DPR_INT16;
    typedef int32_t     DPR_INT32;

    /* helper macros */
    #define DPR_PTR_TO_ULONG(x)                ((unsigned long)(x))
    #define DPR_PTR_TO_INT(x)                  ((int)(x))
    /* user specific file operations / types */
    typedef FILE*     DPR_DRV_HANDLE;
    #define DPR_DRV_OPEN(path) (fopen(path, "r+"))
    #define DPR_DRV_CLOSE(stream) (fclose(stream))
    #define DPR_DRV_WRITE(file, buf, nbyte) (write(fileno(file), buf, nbyte))
    #define DPR_DRV_READ(file, buf, nbyte) (read(fileno(file), buf, nbyte))
    #define DPR_DRV_ERROR(file)  ferror(file)
    #define DPR_DRV_IOCTL(file, cmd, arg, size_in, siz_out) (ioctl(fileno(file), cmd, arg))
    #define DPR_DRV_MMAP(file, off, len, usewr_id) (mmap(0, len, PROT_READ|PROT_WRITE, MAP_SHARED, fileno(file), off))
    #define DPR_DRV_MUNMAP(file, ptr, len, user_id) (munmap(ptr, len))
    #define DPR_DRV_UTCTIME       GetSystemSeconds
    #define DPR_DRV_CONFIG_PATH   GetConfigPath
    #define DPR_STRERROR() strerror(errno)

    #ifndef O_BINARY
     #define O_BINARY 0
    #endif

    /* user semaphore API (pthread) */
    typedef sem_t DPR_SEMAPHORE;
    #define DPR_SEM_CREATE(semObj) (sem_init(&(semObj), 0, 0))
    #define DPR_SEM_WAIT(semObj) (sem_wait(&(semObj)))
    #define DPR_SEM_POST(semObj) (sem_post(&(semObj)))
    #define DPR_SEM_DESTROY(semObj) (sem_destroy(&(semObj)))

    #define DPR_SEM_RET_SIGNALED 0
    #define DPR_SEM_RET_TIMEOUT  1

    __inline int sem_wait_time(DPR_SEMAPHORE *semObj, DPR_UINT32 msecs)
    {
      int semret;
      long wait_step_us = 1000; /*(msecs > 10) ? (5*1000) : (1*1000);*/
      long wait_time_us = (long) (msecs * 1000);

      while((semret = sem_trywait(semObj)) != 0 && (wait_time_us > 0)){
        usleep(wait_step_us);
        wait_time_us -= wait_step_us;
        /* printf("DPR_SEM_WAIT_TIME wait_time_us=%lu\n", wait_time_us); */
      }

      if(semret == 0){
        /* printf("DPR_SEM_WAIT_TIME waited_time_ms=%lu\n", (msecs * 1000 - wait_time_us)/1000); */
        return DPR_SEM_RET_SIGNALED;
      }
      else {
        return DPR_SEM_RET_TIMEOUT;
      }
    }

    #define DPR_SEM_WAIT_TIME(semObj, msecs) sem_wait_time(&(semObj), msecs)

    /* user mutex API interprocess! (pthread) */
    struct interprocess_mutex {
        pthread_mutex_t mux;
    };

    inline struct interprocess_mutex *UserInterprocessMutexCreate(int id)
    {
        int shmid, creator;
        size_t size = sizeof (struct interprocess_mutex);
        struct interprocess_mutex *mux;

        creator = 1;

        shmid = shmget(0x1616 + id, size, IPC_CREAT | IPC_EXCL | 0666);
        if(-1 == shmid && EEXIST == errno) {
            shmid = shmget(0x1616 + id, size, 0666);
            creator = 0;
        }

        if(-1 == shmid) {
            return NULL;
        }

        mux = (struct interprocess_mutex *)shmat(shmid, NULL, 0);
        if(-1 == (int)(long)mux) {
            return NULL;
        }

        if(creator) {
            pthread_mutexattr_t mattr;
            int ret;

            ret = pthread_mutexattr_init(&mattr);
            assert(!ret && 1 == 1);
            ret = pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
            if(ENOTSUP != ret && ENOSYS != ret)
                assert(!ret && 2 == 2);
            ret = pthread_mutex_init(&mux->mux, &mattr);
            assert(!ret && 3 == 3);
            ret = pthread_mutexattr_destroy(&mattr);
            assert(!ret && 4 == 4);
        }

        return mux;
    };

    inline void UserInterprocessMutexDestroy(struct interprocess_mutex *mux, int id)
    {
        shmdt(mux);
    };

    typedef struct interprocess_mutex *DPR_INTERPROCESS_MUTEX;
    #define DPR_INTERPROCESS_MUTEX_CREATE_UNLOCKED(iMuxObj, id) (!(iMuxObj = UserInterprocessMutexCreate(id)))
    #define DPR_INTERPROCESS_MUTEX_LOCK(iMuxObj) pthread_mutex_lock(&iMuxObj->mux)
    #define DPR_INTERPROCESS_MUTEX_UNLOCK(iMuxObj) pthread_mutex_unlock(&iMuxObj->mux)
    #define DPR_INTERPROCESS_MUTEX_DESTROY(iMuxObj, id) UserInterprocessMutexDestroy(iMuxObj, id)

    /* user mutex API (pthread) */
    typedef pthread_mutex_t DPR_MUTEX;
    #define DPR_MUTEX_CREATE_UNLOCKED(muxObj) (pthread_mutex_init(&(muxObj), NULL))
    #define DPR_MUTEX_LOCK(muxObj) (pthread_mutex_lock(&(muxObj)))
    #define DPR_MUTEX_UNLOCK(muxObj) (pthread_mutex_unlock(&(muxObj)))
    #define DPR_MUTEX_DESTROY(muxObj) (pthread_mutex_destroy(&(muxObj)))

    /* user time API */
    #define DPR_TASK_DELAY(msecs) (usleep(1000*(msecs)))
    #define DPR_GMTIME(a, b) gmtime_r(a, b)

    /* user thread API (pthread) */
    typedef pthread_t DPR_THREAD_HANDLE;
    typedef void* DPR_THREAD_RETURN_TYPE;
    #define DPR_THREAD_DECL
    inline int DPR_THREAD_CREATE(DPR_THREAD_HANDLE * tHndl, const char *name, void *(*ptFunc) (void *),
        DPR_VOID * pParams)
    {
        pthread_attr_t temptAttribute;
        int ret;

        /* create threads */
        pthread_attr_init(&temptAttribute);
        pthread_attr_setschedpolicy(&temptAttribute, SCHED_OTHER);
        if((ret = pthread_create(tHndl, &temptAttribute, ptFunc, pParams)) != 0) {
            printf(" * UserCreateThread: Thread creation failed %d\n", ret);
            return 0;
        }

        return 1;
    }

    #define DPR_THREAD_JOIN(tHndl) (pthread_join((DPR_THREAD_HANDLE)tHndl, NULL))
    #define DPR_THREAD_END() pthread_exit(NULL)
    #define DPR_GET_CURRENT_PROCESS_ID() getpid()
    #define DPR_GET_CURRENT_THREAD_ID() pthread_self()

    /* memory allocation API */
    #define DPR_VMALLOC(size) malloc(size)
    #define DPR_VFREE(ptr) free(ptr)
    #define DPR_VREALLOC(ptr, size) realloc((ptr), (size))

    #define DPR_ZALLOC(size) calloc(size, 1)
    #define DPR_FREE(mObj) free(mObj)

    /* page size */
    #define DPR_GET_PAGE_SIZE getpagesize()

    /* user assert API */
    #define DPR_ASSERT(x) assert(x)
    #define ASSERT(x) DPR_ASSERT(x)

    static inline DPR_UINT32 GetSystemSeconds(void)
    {
      struct timespec tm;

      clock_gettime(CLOCK_REALTIME, &tm);

      return (DPR_UINT32)tm.tv_sec;
    }

    static inline const char *GetConfigPath(void)
    {
      return "/etc/16xx_config";
    }

#endif /* ifndef __KERNEL__ */

#define DPR_CONTROL_INTERFACE "/dev/cp16xx%u/control"
#define DPR_SYNC_INTERFACE    "/dev/cp16xx%u/synch"
#define DPR_ALARM_INTERFACE   "/dev/cp16xx%u/alarm"
#define DPR_MODEIND_INTERFACE "/dev/cp16xx%u/modind"
#define DPR_RECORDS_INTERFACE "/dev/cp16xx%u/datarec"
#define DPR_MGT_INTERFACE     "/dev/cp16xx%u/mgt"
#define DPR_L2_SEND_INTERFACE "/dev/cp16xx%u/l2eth_send"
#define DPR_L2_RECV_INTERFACE "/dev/cp16xx%u/l2eth_receive"

/* API uses 1-based index; Driver, API internals and
    interfaces uses 0- based */
#define DRIVER_IDX(x) ((x) - 1)

/* ioctl API */
/* consult linux/Documentation/ioctl-number.txt for preffered value */
#define CP16XX_IOC_MAGIC 0xA1

#define IOCTLNUMBER_NOIO(_nr)          _IO(CP16XX_IOC_MAGIC, _nr)
#define IOCTLNUMBER_ROIO(_nr, _object) _IOR(CP16XX_IOC_MAGIC, _nr, _object)
#define IOCTLNUMBER_WOIO(_nr, _object) _IOW(CP16XX_IOC_MAGIC, _nr, _object)
#define IOCTLNUMBER_RWIO(_nr, _object) _IOWR(CP16XX_IOC_MAGIC, _nr, _object)

#if defined(RTAI)
#define RTAI_MAGIC_STARTOP  "str"
#define RTAI_MAGIC_OPFAULT  "flt"
#define RTAI_MAGIC_NEWCYCLE "ncy"

#define RTAI_UNIC_NAME(tmp, x, y) (sprintf(tmp, "%s%d", (x), y), tmp)
#elif defined(XENOMAI)
#define XENO_MAGIC_STARTOP  "str"
#define XENO_MAGIC_OPFAULT  "flt"
#define XENO_MAGIC_NEWCYCLE "ncy"

#define XENO_UNIC_NAME(tmp, x, y) (sprintf(tmp, "%s%d", (x), y), tmp)
#endif

#endif /* _OS_LINUX_H */
