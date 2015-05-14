/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx_ioctl32.c
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

#include "../os.h"        /* system dependencies */
#include "../cp16xx.h"    /* common(to user and kernel spaces) structures and defines */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)

/* compatibility cruft,
   SPARC running mixed 32 bits in user cpace/64 bits in kernel enviroinment */
#ifdef CONFIG_COMPAT

/* compatibility structs, ioctls and routines */
struct t_rw_direct32 {
    u32  offset;  /* in */
    u32  length;  /* in/out */
    u32  data;    /* in/out */
};

#define CP16XX_IOCREGR32   _IOWR(CP16XX_IOC_MAGIC, 2, struct t_rw_direct32)
#define CP16XX_IOCREGW32    _IOW(CP16XX_IOC_MAGIC, 3, struct t_rw_direct32)
#define CP16XX_IOCDPRAMR32 _IOWR(CP16XX_IOC_MAGIC, 4, struct t_rw_direct32)
#define CP16XX_IOCDPRAMW32  _IOW(CP16XX_IOC_MAGIC, 5, struct t_rw_direct32)
#define CP16XX_IOCDMAR32   _IOWR(CP16XX_IOC_MAGIC, 7, struct t_rw_direct32)
#define CP16XX_IOCDMAW32    _IOW(CP16XX_IOC_MAGIC, 8, struct t_rw_direct32)

static int get_rw_direct32(struct t_rw_direct *dst, struct t_rw_direct32 *src)
{
    u32 data32;
    unsigned long data;
    void *datap;

    if(get_user(data32, &src->offset))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->offset))
        return -EFAULT;

    DPRLIBLOGMSG("offset: src 0x%x, dst 0x%lx\n", data32, data);

    if(get_user(data32, &src->length))
        return -EFAULT;

    data = (unsigned long)data32;
    if(put_user(data, &dst->length))
        return -EFAULT;

    DPRLIBLOGMSG("length: src 0x%x, dst 0x%lx\n", data32, data);

    if(get_user(data32, &src->data))
        return -EFAULT;

    datap = (void* )(unsigned long)data32;
    if(put_user(datap, &dst->data))
        return -EFAULT;

    DPRLIBLOGMSG("data: src 0x%x, pointer 0x%p\n", data32, datap);

    return 0;
}

static int put_rw_direct32(struct t_rw_direct32 *dst, struct t_rw_direct *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data, &src->offset))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->offset))
        return -EFAULT;

    if(get_user(data, &src->length))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->length))
        return -EFAULT;

    return 0;
}

struct t_dma_address32 {
    u32 dma_address;  /* in/out */
    u32 dma_size;     /* in/out */
};

#define CP16XX_IOC_GET_L2_DMA_PHYS_ADDR32  _IOWR(CP16XX_IOC_MAGIC, 9, struct t_dma_address32)
#define CP16XX_IOC_GET_PNIO_DMA_RANGE32    _IOWR(CP16XX_IOC_MAGIC, 24, struct t_dma_address32)

static int get_dma_address32(struct t_dma_address *dst, struct t_dma_address32 *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data32, &src->dma_address))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->dma_address))
        return -EFAULT;

    DPRLIBLOGMSG("dma_address: src 0x%x, dst 0x%lx\n", data32, data);

    if(get_user(data32, &src->dma_size))
        return -EFAULT;

    data = (unsigned long)data32;
    if(put_user(data, &dst->dma_size))
        return -EFAULT;

    DPRLIBLOGMSG("dma_size: src 0x%x, dst 0x%lx\n", data32, data);

    return 0;
}

static int put_dma_address32(struct t_dma_address32 *dst, struct t_dma_address *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data, (unsigned long *)&src->dma_address))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->dma_address))
        return -EFAULT;

    if(get_user(data, &src->dma_size))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->dma_size))
        return -EFAULT;

    return 0;
}

struct t_register_app32 {
    u32 user_id;  /* in/out */
    u32 flags; /* in */
};

#define CP16XX_IOC_OAPP32   _IOWR(CP16XX_IOC_MAGIC, 10, struct t_register_app32)
#define CP16XX_IOC_CAPP32   _IOW(CP16XX_IOC_MAGIC, 11, struct t_register_app32)
#define CP16XX_IOC_IRTCBF32 _IOW(CP16XX_IOC_MAGIC, 14, struct t_register_app32)
#define CP16XX_IOC_IRTCPID32 _IOW(CP16XX_IOC_MAGIC, 39, struct t_register_app32)

static int get_register_app32(struct t_register_app *dst, struct t_register_app32 *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data32, &src->user_id))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->user_id))
        return -EFAULT;

    if(get_user(data32, &src->flags))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->flags))
        return -EFAULT;

    return 0;
}

static int put_register_app32(struct t_register_app32 *dst, struct t_register_app *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data, &src->user_id))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->user_id))
        return -EFAULT;

    if(get_user(data, &src->flags))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->flags))
        return -EFAULT;

    return 0;
}

struct t_dma_range32 {
    u32 user_id;    /* in */
    u32 offset_in;  /* in */
    u32 length_in;  /* in */
    u32 offset_out; /* in */
    u32 length_out; /* in */
};

#define CP16XX_IOC_SET_DMA_RANGE32 _IOW(CP16XX_IOC_MAGIC, 15, struct t_dma_range32)

static int get_dma_range32(struct t_dma_range *dst, struct t_dma_range32 *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data32, &src->user_id))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->user_id))
        return -EFAULT;

    if(get_user(data32, &src->offset_in))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->offset_in))
        return -EFAULT;

    if(get_user(data32, &src->length_in))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->length_in))
        return -EFAULT;

    if(get_user(data32, &src->offset_out))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->offset_out))
        return -EFAULT;

    if(get_user(data32, &src->length_out))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->length_out))
        return -EFAULT;

    return 0;
}

struct t_read_timer32 {
    u32 cp_timer;  /* out */
    /* propably more params later */
};

#define CP16XX_IOC_GET_CP_TIMER32  _IOR(CP16XX_IOC_MAGIC, 16, struct t_read_timer32)

static int put_read_timer32(struct t_read_timer32 *dst, struct t_read_timer *src)
{
    unsigned long data32;
    u32 data;

    if(get_user(data32, &src->cp_timer))
        return -EFAULT;
    data = (u32)data32;
    if(put_user(data, &dst->cp_timer))
        return -EFAULT;

    return 0;
}

struct t_read_pool32 {
    u32 user_id;  /* in */
    u32 read_length; /* out */
    u32 write_length; /* out */
    u32 reserved1;
    u32 reserved2;
};

#define CP16XX_IOC_BIND32   _IOWR(CP16XX_IOC_MAGIC, 12, struct t_read_pool32)
#define CP16XX_IOC_UNBIND32 _IOW(CP16XX_IOC_MAGIC, 13, struct t_read_pool32)

static int get_read_pool32(struct t_read_pool *dst, struct t_read_pool32 *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data32, &src->user_id))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->user_id))
        return -EFAULT;

    return 0;
}

static int put_read_pool32(struct t_read_pool32 *dst, struct t_read_pool *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data, &src->user_id))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->user_id))
        return -EFAULT;

    if(get_user(data, &src->read_length))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->read_length))
        return -EFAULT;

    if(get_user(data, &src->write_length))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->write_length))
        return -EFAULT;

    return 0;
}

struct t_dma_pool32 {
    u32 offset;       /* in */
    u8 data[MAX_ETHERNET_SIZE_FRAME];        /* in/out */
    u32 length;       /* in/out */
};

#define CP16XX_IOC_PACKET_READ32   _IOWR(CP16XX_IOC_MAGIC, 18, struct t_dma_pool32)
#define CP16XX_IOC_PACKET_WRITE32  _IOWR(CP16XX_IOC_MAGIC, 19, struct t_dma_pool32)

static int get_dma_pool32(struct t_dma_pool *dst, struct t_dma_pool32 *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data32, &src->offset))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->offset))
        return -EFAULT;

    if(DPR_MEMCPY_FROM_USER(dst->data, src->data, MAX_ETHERNET_SIZE_FRAME))
        return -EFAULT;

    if(get_user(data32, &src->length))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->length))
        return -EFAULT;

    return 0;
}

static int put_dma_pool32(struct t_dma_pool32 *dst, struct t_dma_pool *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data, &src->offset))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->offset))
        return -EFAULT;

    if(DPR_MEMCPY_TO_USER(dst->data, src->data, MAX_ETHERNET_SIZE_FRAME))
        return -EFAULT;

    if(get_user(data, &src->length))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->length))
        return -EFAULT;

    return 0;
}

struct t_register_appl_wd32 {
    u32 user_id;  /* in, now unused */
    u32   wd_id;  /* in/out */
    u32 timeout;  /* in */
};

#define CP16XX_IOC_OWD32   _IOWR(CP16XX_IOC_MAGIC, 20, struct t_register_appl_wd32)
#define CP16XX_IOC_CWD32    _IOW(CP16XX_IOC_MAGIC, 21, struct t_register_appl_wd32)
#define CP16XX_IOC_TWD32    _IOW(CP16XX_IOC_MAGIC, 22, struct t_register_appl_wd32)

static int get_register_appl_wd32(struct t_register_appl_wd *dst, struct t_register_appl_wd32 *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data32, &src->user_id))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->user_id))
        return -EFAULT;

    if(get_user(data32, &src->wd_id))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->wd_id))
        return -EFAULT;

    if(get_user(data32, &src->timeout))
        return -EFAULT;
    data = (unsigned long)data32;
    if(put_user(data, &dst->timeout))
        return -EFAULT;

    return 0;
}

static int put_register_appl_wd32(struct t_register_appl_wd32 *dst, struct t_register_appl_wd *src)
{
    u32 data32;
    unsigned long data;

    if(get_user(data, &src->user_id))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->user_id))
        return -EFAULT;

    if(get_user(data, &src->wd_id))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->wd_id))
        return -EFAULT;

    if(get_user(data, &src->timeout))
        return -EFAULT;
    data32 = (u32)data;
    if(put_user(data32, &dst->timeout))
        return -EFAULT;

    return 0;
}

static int cp16xx_ioctl32_compatible(struct file *file, unsigned int cmd,
    unsigned long arg)
{
#ifdef HAVE_UNLOCKED_IOCTL
    if(! file->f_op || ! file->f_op->compat_ioctl)
      return -ENOTTY;
    return file->f_op->compat_ioctl(file, cmd, arg);
#else
    if(! file->f_op || ! file->f_op->ioctl)
      return -ENOTTY;
    return file->f_op->ioctl(file->f_dentry->d_inode, file, cmd, arg);
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#if defined(CONFIG_SPARC64)
static __inline__ void *compat_alloc_user_space(long len)
{
    struct pt_regs *regs = current_thread_info()->kregs;
    unsigned long usp = regs->u_regs[UREG_I6];

    if (!(test_thread_flag(TIF_32BIT)))
        usp += STACK_BIAS;

    return (void *) (usp - len);
}
#elif defined(CONFIG_X86_64)
static __inline__ void *compat_alloc_user_space(long len)
{
    struct pt_regs *regs = (void *)current_thread.rsp0 - sizeof(struct pt_regs);
    return (void *)regs->rsp - len;
}
#else
 #error please provide arch depeneded compat_alloc_user_space function()
#endif
#else
#include <linux/compat.h>
#endif

/* always allocate user stack */
static int cp16xx_ioctl32_not_compatible(struct file *file, unsigned int cmd,
    unsigned long arg)
{
    union {
        struct t_rw_direct *rw_direct;
        struct t_dma_address *dma_address;
        struct t_register_app *register_app;
        struct t_dma_range *dma_range;
        struct t_read_timer *read_timer;
        struct t_read_pool *read_pool;
        struct t_dma_pool *dma_pool;
        struct t_register_appl_wd *register_appl_wd;
        void *arg64;
    } args;

    int err = 0;

    DPRLIBLOGMSG("begin cmd 0x%x, arg 0x%lx\n", cmd, arg);

    switch(cmd) {
    case CP16XX_IOCREGR32: cmd = CP16XX_IOCREGR; break;
    case CP16XX_IOCREGW32: cmd = CP16XX_IOCREGW; break;
    case CP16XX_IOCDPRAMR32: cmd = CP16XX_IOCDPRAMR; break;
    case CP16XX_IOCDPRAMW32: cmd = CP16XX_IOCDPRAMW; break;
    case CP16XX_IOCDMAR32: cmd = CP16XX_IOCDMAR; break;
    case CP16XX_IOCDMAW32: cmd = CP16XX_IOCDMAW; break;

    case CP16XX_IOC_GET_L2_DMA_PHYS_ADDR32: cmd = CP16XX_IOC_GET_L2_DMA_PHYS_ADDR; break;
    case CP16XX_IOC_GET_PNIO_DMA_RANGE32: cmd = CP16XX_IOC_GET_PNIO_DMA_RANGE; break;

    case CP16XX_IOC_OAPP32: cmd = CP16XX_IOC_OAPP; break;
    case CP16XX_IOC_CAPP32: cmd = CP16XX_IOC_CAPP; break;
    case CP16XX_IOC_IRTCBF32: cmd = CP16XX_IOC_IRTCBF; break;
    case CP16XX_IOC_IRTCPID32: cmd = CP16XX_IOC_IRTCPID; break;

    case CP16XX_IOC_SET_DMA_RANGE32: cmd = CP16XX_IOC_SET_DMA_RANGE; break;

    case CP16XX_IOC_GET_CP_TIMER32: cmd = CP16XX_IOC_GET_CP_TIMER; break;

    case CP16XX_IOC_BIND32: cmd = CP16XX_IOC_BIND; break;
    case CP16XX_IOC_UNBIND32: cmd = CP16XX_IOC_UNBIND; break;

    case CP16XX_IOC_PACKET_READ32: cmd = CP16XX_IOC_PACKET_READ; break;
    case CP16XX_IOC_PACKET_WRITE32: cmd = CP16XX_IOC_PACKET_WRITE; break;

    case CP16XX_IOC_OWD32: cmd = CP16XX_IOC_OWD; break;
    case CP16XX_IOC_CWD32: cmd = CP16XX_IOC_CWD; break;
    case CP16XX_IOC_TWD32: cmd = CP16XX_IOC_TWD; break;

    default:
        DPRLIBERRMSG("no conversion for this ioctl 0x%x\n", cmd);
        return -ENOTTY;
    };

    switch(cmd) {
    case CP16XX_IOCREGR:
    case CP16XX_IOCREGW:
    case CP16XX_IOCDPRAMR:
    case CP16XX_IOCDPRAMW:
    case CP16XX_IOCDMAR:
    case CP16XX_IOCDMAW:
        args.rw_direct = compat_alloc_user_space(sizeof(*args.rw_direct));
        err = get_rw_direct32(args.rw_direct, (struct t_rw_direct32 *)arg);
        break;
    case CP16XX_IOC_GET_L2_DMA_PHYS_ADDR:
    case CP16XX_IOC_GET_PNIO_DMA_RANGE:
        args.dma_address = compat_alloc_user_space(sizeof(*args.dma_address));
        err = get_dma_address32(args.dma_address, (struct t_dma_address32 *)arg);
        break;
    case CP16XX_IOC_OAPP:
    case CP16XX_IOC_IRTCPID:
        args.register_app = compat_alloc_user_space(sizeof(*args.register_app));
        err = get_register_app32(args.register_app, (struct t_register_app32 *)arg);
        break;
    case CP16XX_IOC_CAPP:
    case CP16XX_IOC_IRTCBF:
        args.register_app = compat_alloc_user_space(sizeof(*args.register_app));
        err = get_register_app32(args.register_app, (struct t_register_app32 *)arg);
        break;
    case CP16XX_IOC_GET_CP_TIMER:
        args.read_timer = compat_alloc_user_space(sizeof(*args.read_timer));
        break;
    case CP16XX_IOC_SET_DMA_RANGE:
        args.dma_range = compat_alloc_user_space(sizeof(*args.dma_range));
        err = get_dma_range32(args.dma_range, (struct t_dma_range32 *)arg);
        break;
    case CP16XX_IOC_BIND:
    case CP16XX_IOC_UNBIND:
        args.read_pool = compat_alloc_user_space(sizeof(*args.read_pool));
        err = get_read_pool32(args.read_pool, (struct t_read_pool32 *)arg);
        break;
    case CP16XX_IOC_PACKET_READ:
    case CP16XX_IOC_PACKET_WRITE:
        args.dma_pool = compat_alloc_user_space(sizeof(*args.dma_pool));
        err = get_dma_pool32(args.dma_pool, (struct t_dma_pool32 *)arg);
        break;
    case CP16XX_IOC_OWD:
    case CP16XX_IOC_CWD:
    case CP16XX_IOC_TWD:
        args.register_appl_wd = compat_alloc_user_space(sizeof(*args.register_appl_wd));
        err = get_register_appl_wd32(args.register_appl_wd, (struct t_register_appl_wd32 *)arg);
        break;
    }
    if(err)
        goto do_cp16xx_ioctl_out;

#ifdef HAVE_UNLOCKED_IOCTL
    err = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)args.arg64);
#else
    err = file->f_op->ioctl(file->f_dentry->d_inode, file, cmd, (unsigned long)args.arg64);
#endif

    if(err == 0) {
        switch(cmd) {
        case CP16XX_IOCREGR:
        case CP16XX_IOCDPRAMR:
        case CP16XX_IOCDMAR:
            err = put_rw_direct32((struct t_rw_direct32 *)arg, args.rw_direct);
            break;
        case CP16XX_IOC_GET_L2_DMA_PHYS_ADDR:
        case CP16XX_IOC_GET_PNIO_DMA_RANGE:
            err = put_dma_address32((struct t_dma_address32 *)arg, args.dma_address);
            break;
        case CP16XX_IOC_OAPP:
        case CP16XX_IOC_IRTCPID:
            err = put_register_app32((struct t_register_app32 *)arg, args.register_app);
            break;
        case CP16XX_IOC_GET_CP_TIMER:
            err = put_read_timer32((struct t_read_timer32 *)arg, args.read_timer);
            break;
        case CP16XX_IOC_BIND:
            err = put_read_pool32((struct t_read_pool32 *)arg, args.read_pool);
            break;
        case CP16XX_IOC_PACKET_READ:
        case CP16XX_IOC_PACKET_WRITE:
            err = put_dma_pool32((struct t_dma_pool32 *)arg, args.dma_pool);
            break;
        case CP16XX_IOC_OWD:
            err = put_register_appl_wd32((struct t_register_appl_wd32 *)arg, args.register_appl_wd);
            break;
        }
    }
do_cp16xx_ioctl_out:
    DPRLIBLOGMSG("end err %d\n", err);
    return err;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
static inline int cp16xx_ioctl32_compatible_as_is(unsigned int fd, unsigned int cmd,
    unsigned long arg, struct file *file)
{
    return cp16xx_ioctl32_compatible(file, cmd, arg);
}

static inline int cp16xx_ioctl32_not_compatible_as_is(unsigned int fd, unsigned int cmd,
    unsigned long arg, struct file *file)
{
    return cp16xx_ioctl32_not_compatible(file, cmd, arg);
}

extern int register_ioctl32_conversion(unsigned int cmd,
    int (*handler)(unsigned int, unsigned int, unsigned long, struct file *));
extern int unregister_ioctl32_conversion(unsigned int cmd);

void cp16xx_register_ioctl32_conversion(void)
{
    int err = 0;

    DPRLIBLOGMSG("begin\n");

    err += register_ioctl32_conversion(CP16XX_IOCSHUTDOWN, cp16xx_ioctl32_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCRESET, cp16xx_ioctl32_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCWIRQ, cp16xx_ioctl32_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCCOUNT, cp16xx_ioctl32_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOCREGR32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCREGW32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCDPRAMR32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCDPRAMW32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCDMAR32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOCDMAW32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_GET_L2_DMA_PHYS_ADDR32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_GET_PNIO_DMA_RANGE32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_OAPP32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_CAPP32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_IRTCBF32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_IRTCPID32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_SET_DMA_RANGE32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_GET_CP_TIMER32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_BIND32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_UNBIND32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_PACKET_READ32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_PACKET_WRITE32, cp16xx_ioctl32_not_compatible_as_is);

    err += register_ioctl32_conversion(CP16XX_IOC_OWD32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_CWD32, cp16xx_ioctl32_not_compatible_as_is);
    err += register_ioctl32_conversion(CP16XX_IOC_TWD32, cp16xx_ioctl32_not_compatible_as_is);

    DPRLIBLOGMSG("end err %d\n", err);
}

void cp16xx_unregister_ioctl32_conversion(void)
{
    int err = 0;

    DPRLIBLOGMSG("begin\n");

    err += unregister_ioctl32_conversion(CP16XX_IOCSHUTDOWN);
    err += unregister_ioctl32_conversion(CP16XX_IOCRESET);
    err += unregister_ioctl32_conversion(CP16XX_IOCWIRQ);
    err += unregister_ioctl32_conversion(CP16XX_IOCCOUNT);

    err += unregister_ioctl32_conversion(CP16XX_IOCREGR32);
    err += unregister_ioctl32_conversion(CP16XX_IOCREGW32);
    err += unregister_ioctl32_conversion(CP16XX_IOCDPRAMR32);
    err += unregister_ioctl32_conversion(CP16XX_IOCDPRAMW32);
    err += unregister_ioctl32_conversion(CP16XX_IOCDMAR32);
    err += unregister_ioctl32_conversion(CP16XX_IOCDMAW32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_GET_L2_DMA_PHYS_ADDR32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_GET_PNIO_DMA_RANGE32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_OAPP32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_CAPP32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_IRTCBF32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_IRTCPID32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_SET_DMA_RANGE32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_GET_CP_TIMER32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_BIND32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_UNBIND32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_PACKET_READ32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_PACKET_WRITE32);

    err += unregister_ioctl32_conversion(CP16XX_IOC_OWD32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_CWD32);
    err += unregister_ioctl32_conversion(CP16XX_IOC_TWD32);

    DPRLIBLOGMSG("end err %d\n", err);
}

#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) */

long cp16xx_ioctl_compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch(cmd) {
    case CP16XX_IOCSHUTDOWN:
    case CP16XX_IOCRESET:
    case CP16XX_IOCWIRQ:
    case CP16XX_IOCCOUNT:
        return cp16xx_ioctl32_compatible(filp, cmd, arg);
    default:
        return cp16xx_ioctl32_not_compatible(filp, cmd, arg);
    }

    return -ENOTTY;
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) */

#endif /* CONFIG_COMPAT */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0) */
