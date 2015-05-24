/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/* FILE NAME    : cp16xx.h
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

#ifndef _CP16XX_H
#define _CP16XX_H

#ifdef WIN32 // for Windows read and write calls lib-> driver over ioctrl
#define READ_OVER_IOCTL // read calls lib-> driver over ioctrl instead of readfile
#define RD_WR_OVER_IOCTL // write calls lib-> driver over ioctrl instead of readfile
#endif
#ifdef WIN64 
#define READ_OVER_IOCTL // read calls lib-> driver over ioctrl instead of readfile
#define RD_WR_OVER_IOCTL // write calls lib-> driver over ioctrl instead of readfile
#endif



/* ERTEC constans*/
#define SWI_CTRL                0x00019038
#define SWI_VERSION             0x00019400

/* ERTEC IRQ control registers */
#define HP_IRQ_MODE             0x00017000

#define BIT_IRQ_NEWCYCLE        (1 << 0)
#ifdef LATENCY_MEASUREMENT
  /* use cycle irq as source */
  #define BIT_IRQ_STARTOP         (1 << 0)
#else
  #define BIT_IRQ_STARTOP         (1 << 1)
#endif

#define BIT_IRQ_DMAIN           (1 << 3)
#define BIT_IRQ_DMAOUT          (1 << 4)
#define BIT_IRQ_INFAULT         (1 << 5)
#define BIT_IRQ_OUTFAULT        (1 << 6)
#define BIT_IRQ_HP              (1 << 12)

#define OPFAULT_BITS (BIT_IRQ_DMAIN | BIT_IRQ_DMAOUT | BIT_IRQ_INFAULT | BIT_IRQ_OUTFAULT)

/* mask bitsets */
#define HP_IRQ0_MASK_IRT        0x00017004
#define HP_IRQ0_MASK_RT         0x00017008
#define HP_IRQ1_MASK_IRT        0x0001700C
#define HP_IRQ1_MASK_RT         0x00017010

/* read here the IRQ reason */
#define HP_IRQ0_IRT             0x00017400
#define HP_IRQ0_RT              0x00017404
#define HP_IRQ1_IRT             0x00017408
#define HP_IRQ1_RT              0x0001740C

/* write here the acknowledeg bit */
#define HP_IRQ_ACK_IRT          0x00017410
#define HP_IRQ_ACK_RT           0x00017414

/* end of IRQ registers */
#define HP_EOI_IRQ0             0x00017420
#define HP_EOI_IRQ1             0x00017424

#define HP_IRQ_ACTIVATE         0x00017450
#define SP_IRQ_ACTIVATE         0x00017454

/* ertec switch trace diag registers */

#define HP_TRACE_MODE           0x00018404
#define HP_TRACE_BASE_ADDR      0x00018000
#define HP_TRACE_BUF_LEN        0x00018004
#define HP_TRACE_BUF_ENTRY      0x00018408
#define HP_TRACE_FAULT_CNT      0x0001840C
#define HP_TRACE_TRANS_LIM      0x00018008
#define HP_TRACE_COMMAND        0x00018400

#define HP_TRACE_DIAG_ENABLE_P0 0x00000008
#define HP_TRACE_DIAG_ENABLE_P1 0x00001008
#define HP_TRACE_DIAG_ENABLE_P2 0x00002008
#define HP_TRACE_DIAG_ENABLE_P3 0x00003008


/* sync mode */

#define DMA_IN_DEST_ADR_0       0x00013028
#define DMA_IN_DEST_ADR_1       0x0001302C
#define DMA_IN_SRC_ADR          0x00013030
#define DMA_IN_LENGTH           0x00013034

#define DMA_OUT_SRC_ADR_0       0x00013038
#define DMA_OUT_SRC_ADR_1       0x0001303C
#define DMA_OUT_DEST_ADR        0x00013040
#define DMA_OUT_LENGTH          0x00013044

#define IRT_SYNCMODE            0x00013400

/* iso-cycle properties */
#define HP_CYCLE_CNT_VAL        0x00011400
#define HP_CYCLE_TIME_VAL       0x00011404
#define HP_CLOCK_CNT_VAL        0x00011414
#define HP_CYCLE_CNT_ENT        0x00011038

/* PCI resources */
#define MMAP_OFFSET_EMIF        0x00000000
#define MMAP_OFFSET_BOOTROM     0x10000000
#define MMAP_OFFSET_RAM         0x20000000
#define MMAP_OFFSET_IRTE        0x30000000
#define MMAP_OFFSET_EMIF_CS0    0x40000000
#define MMAP_OFFSET_DPRAM       0x50000000

/* host resources */
#define MMAP_OFFSET_DMA         0x60000000
#define MMAP_OFFSET_L2_DMA      0x70000000

/* details of MMAP_OFFSET_IRTE */
#define OFFSET_ERTEC_BASE       0x00000000
#define SIZE_ERTEC_BASE         0x00100000
#define OFFSET_IO_TOTAL         0x00100000
#define SIZE_IO_TOTAL           0x00010000
#define MAX_SIZE_IRTE           0x00200000

/* details of MMAP_OFFSET_EMIF_CS0 */
#define OFFSET_FWFLASH          0x00000000
#define SIZE_FWFLASH            0x00800000
#define MAX_SIZE_EMIF_CS0       0x00800000

/* details of MMAP_OFFSET_DPRAM */
#ifdef IO_ROUTER
    #define OFFSET_KRAMTLB      0x00400000
    #define SIZE_KRAMTLB        0x000E0000
    #define OFFSET_IOCTABLE     0x004E0000
    #define SIZE_IOCTABLE       0x00020000   /* 128kb, see also sdram_mem.h */
#else
    #define OFFSET_KRAMTLB      0x00400000
    #define SIZE_KRAMTLB        0x00100000
#endif
#define OFFSET_EXCEPTION        0x00504100
#define SIZE_EXCEPTION          0x00004000
#define OFFSET_TRACE            0x00508100
#define SIZE_TRACE              0x000f7f00
#define MAX_SIZE_DPRAM          0x00600000

/* details of MMAP_OFFSET_DMA */
#define OFFSET_DMA_IMAGE        0x00000000
#define SIZE_DMA_IMAGE          0x00010000
#define MAX_SIZE_DMA_IMAGE      0x00010000

/* IRT specific defines */
#define CP16XX_STARTOP  1
#define CP16XX_OPFAULT  2
#define CP16XX_OPDONE   3
#define CP16XX_NEWCYCLE 4

struct t_rw_direct {
    unsigned long  offset;  /* in */
    unsigned long  length;  /* in/out */
    unsigned char *  data;  /* in/out */
};

#pragma pack(4)
struct t_dma_address {
    unsigned long dma_size;     /* in/out */
    unsigned char *dma_address;                   /* in/out */
#ifdef __KERNEL__	/* Driver HK 19.11.10 */
#ifndef _WIN64
	unsigned long dummy1;	/* dummy 4 bytes so length of structure fits to 64 Bit driver */
#endif
#else /* Application */
#ifndef WIN64
	unsigned long dummy1;	/* dummy 4 bytes so length of structure fits to 64 Bit driver */
#endif
#endif		/* HK 19.11.10 */
};
#pragma pack()

struct t_register_app {
    unsigned long user_id;  /* in/out */
    unsigned long flags;                          /* in, see below REG_*, RUN_*, DEL_*, KICKUP_* and OAPP_APP_* */
};

#define DEL_STARTOP 0
#define REG_STARTOP 1
#define KICKUP_STARTOP 2
#define RUN_STARTOP 3
#define DEL_OPFAULT 10
#define REG_OPFAULT 11
#define KICKUP_OPFAULT 12
#define RUN_OPFAULT 13
#define DEL_NEWCYCLE 20
#define REG_NEWCYCLE 21
#define KICKUP_NEWCYCLE 22
#define RUN_NEWCYCLE 23

#define OAPP_APP_NORMAL    0
#define OAPP_APP_EXCLUSIVE 1 /* register application with exclusive rigths, only one application for cp */
#define OAPP_APP_L2        2 /* application, that needs bind to L2_SEND/RECV channels */

struct t_dma_range {
    unsigned long user_id;    /* in */
    unsigned long offset_in;  /* in */
    unsigned long length_in;  /* in */
    unsigned long offset_out; /* in */
    unsigned long length_out; /* in */
};

/* used for CP16XX_IOC_PACKET_READ/WRITE can be removed later */
#define MAX_ETHERNET_SIZE_FRAME 1514
struct t_dma_pool {
    unsigned long offset;                         /* in */
    unsigned char data[MAX_ETHERNET_SIZE_FRAME];  /* in/out */
    unsigned long length;                         /* in/out */
};

struct t_read_pool {
    unsigned long user_id;  /* in */
    unsigned long read_length; /* out */
    unsigned long write_length; /* out */
    unsigned long reserved1;
    unsigned long reserved2;
};

struct t_read_timer {
    unsigned long cp_timer;                       /* out */
    /* propably more params later */
};

struct t_register_appl_wd {
    unsigned long user_id;  /* in/out */
    unsigned long   wd_id;  /* in/out */
    unsigned long timeout;  /* in */
    unsigned long read_length; /* out */
    unsigned long reserved1;
    unsigned long reserved2;
};

/* if OS does not provide mmap interface, implement it as IOCTL */
#pragma pack(4)
struct t_mmap_struct {
    unsigned long user_id;                        /* in */
    unsigned long offset;                         /* in */
    unsigned long size;                           /* in */
    unsigned char *user_space_ptr;                /* out */
#ifdef __KERNEL__	/* Driver HK 19.11.10 */
#ifndef _WIN64
	unsigned long dummy1;	/* dummy 4 bytes so length of structure fits to 64 Bit driver */
#endif
#else /* Application */
#ifndef WIN64
	unsigned long dummy1;	/* dummy 4 bytes so length of structure fits to 64 Bit driver */
#endif
#endif		/* HK 19.11.10 */
#pragma pack()
};

enum CP16XX_IOCTLS {
    SHUTDOWN = 0,         /* reset card, if no applications are registered */
    RESET,                /* reset card uncondiniously */
    REGR,                 /* direct read register */
    REGW,                 /* direct write register */
    DPRAMR,               /* direct read DPRAM */
    DPRAMW = 5,           /* direct write DPRAM */
    WIRQ,                 /* wait for RT interrupt */
    DMAR,                 /* direct read DMA */
    DMAW,                 /* direct write DMA */
    GET_L2_DMA_PHYS_ADDR, /* get DMA physical address */
    OAPP = 10,            /* register application */
    CAPP,                 /* unregister application */
    BIND,                 /* bind channel */
    UNBIND,               /* unbind channel */
    IRTCBF,               /* app will use IRT IRQs */
    SET_DMA_RANGE = 15,   /* not used for productive mode,
                             deprecated, allowed only for controller debug purposes*/
    GET_CP_TIMER,         /* read cp16xx timer register */
    COUNT,                /* get debug loop counter of cp16xx_proc */
    PACKET_READ,          /* read dma packet */
    PACKET_WRITE,         /* write dma packet */
    OWD = 20,             /* register new watchdog */
    CWD,                  /* unregister watchdog */
    TWD,                  /* trigger watchdog */
    SWITCH_DIAG,
    GET_PNIO_DMA_RANGE,   /* get DMA physical address */
#if 0 /*def __KERNEL__*/
    /* all this codes are direct ioctl from one modul to another */
    KERN_OAPP = 30,       /* register application */
    KERN_CAPP,            /* unregister application */
    KERN_BIND,            /* bind channel */
    KERN_UNBIND,          /* unbind channel */
    unused_34,
    unused_35 = 35,
    KERN_GET_CP_TIMER,    /* read cards timer register */
    KERN_GET_L2_DMA_PHYS_ADDR, /* get DMA physical address */
    KERN_GET_L2_DMA_VIRT_ADDR, /* get DMA virtual address */
#endif
    IRTCPID = 39,        /* LINUX-RTAI specific due different scan methode, can happen that
                            cp behind interface 0 has card_id 1 and creates
                            RTAI semaphores with suffix 1 */
    MMAP    = 40,        /* map memory, if OS does not provide mmap interface */
    MUNMAP  = 41,        /* unmap memory, if OS does not provide mmap interface */
	WRITEF   = 42,
	READF = 43
};

#ifndef RD_WR_OVER_IOCTL

#define CP16XX_IOCSHUTDOWN              IOCTLNUMBER_NOIO(SHUTDOWN)
#define CP16XX_IOCRESET                 IOCTLNUMBER_NOIO(RESET)
#define CP16XX_IOCREGR                  IOCTLNUMBER_RWIO(REGR, struct t_rw_direct)
#define CP16XX_IOCREGW                  IOCTLNUMBER_WOIO(REGW, struct t_rw_direct)
#define CP16XX_IOCDPRAMR                IOCTLNUMBER_RWIO(DPRAMR, struct t_rw_direct)
#define CP16XX_IOCDPRAMW                IOCTLNUMBER_WOIO(DPRAMW, struct t_rw_direct)
#define CP16XX_IOCWIRQ                  IOCTLNUMBER_NOIO(WIRQ)
#define CP16XX_IOCDMAR                  IOCTLNUMBER_RWIO(DMAR, struct t_rw_direct)
#define CP16XX_IOCDMAW                  IOCTLNUMBER_WOIO(DMAW, struct t_rw_direct)
#define CP16XX_IOC_GET_L2_DMA_PHYS_ADDR IOCTLNUMBER_RWIO(GET_L2_DMA_PHYS_ADDR, struct t_dma_address)
#define CP16XX_IOC_OAPP                 IOCTLNUMBER_RWIO(OAPP, struct t_register_app)
#define CP16XX_IOC_CAPP                 IOCTLNUMBER_WOIO(CAPP, struct t_register_app)
#define CP16XX_IOC_BIND                 IOCTLNUMBER_RWIO(BIND, struct t_read_pool)
#define CP16XX_IOC_UNBIND               IOCTLNUMBER_WOIO(UNBIND, struct t_read_pool)
#define CP16XX_IOC_IRTCBF               IOCTLNUMBER_WOIO(IRTCBF, struct t_register_app)
#define CP16XX_IOC_SET_DMA_RANGE        IOCTLNUMBER_WOIO(SET_DMA_RANGE, struct t_dma_range)
#define CP16XX_IOC_GET_CP_TIMER         IOCTLNUMBER_ROIO(GET_CP_TIMER, struct t_read_timer)
#define CP16XX_IOCCOUNT                 IOCTLNUMBER_ROIO(COUNT, struct t_read_pool)
#define CP16XX_IOC_PACKET_READ          IOCTLNUMBER_RWIO(PACKET_READ, struct t_dma_pool)
#define CP16XX_IOC_PACKET_WRITE         IOCTLNUMBER_RWIO(PACKET_WRITE, struct t_dma_pool)

#define CP16XX_IOC_OWD                  IOCTLNUMBER_RWIO(OWD, struct t_register_appl_wd)
#define CP16XX_IOC_CWD                  IOCTLNUMBER_WOIO(CWD, struct t_register_appl_wd)
#define CP16XX_IOC_TWD                  IOCTLNUMBER_WOIO(TWD, struct t_register_appl_wd)

#define CP16XX_IOC_GET_PNIO_DMA_RANGE   IOCTLNUMBER_RWIO(GET_PNIO_DMA_RANGE, struct t_dma_address)

#define CP16XX_IOC_IRTCPID              IOCTLNUMBER_ROIO(IRTCPID, struct t_register_app)
#define CP16XX_IOC_MMAP                 IOCTLNUMBER_RWIO(MMAP, struct t_mmap_struct)
#define CP16XX_IOC_MUNMAP               IOCTLNUMBER_WOIO(MUNMAP, struct t_mmap_struct)
#define CP16XX_WRITEF					IOCTLNUMBER_NOIO(WRITEF)
#define CP16XX_READF					IOCTLNUMBER_NOIO(READF)

#else

#define CP16XX_IOCSHUTDOWN              IOCTLNUMBER_NOIO(SHUTDOWN)
#define CP16XX_IOCRESET                 IOCTLNUMBER_NOIO(RESET)
#define CP16XX_WRITEF					IOCTLNUMBER_NOIO(WRITEF)
#define CP16XX_READF					IOCTLNUMBER_NOIO(READF)

#define CP16XX_IOCREGR                  IOCTLNUMBER_RWIO(REGR, struct t_rw_direct)
#define CP16XX_IOCREGW                  IOCTLNUMBER_WOIO(REGW, struct t_rw_direct)
#define CP16XX_IOCDPRAMR                IOCTLNUMBER_RWIO(DPRAMR, struct t_rw_direct)
#define CP16XX_IOCDPRAMW                IOCTLNUMBER_WOIO(DPRAMW, struct t_rw_direct)
#define CP16XX_IOCWIRQ                  IOCTLNUMBER_NOIO(WIRQ)
#define CP16XX_IOCDMAR                  IOCTLNUMBER_RWIO(DMAR, struct t_rw_direct)
#define CP16XX_IOCDMAW                  IOCTLNUMBER_WOIO(DMAW, struct t_rw_direct)
#define CP16XX_IOC_GET_L2_DMA_PHYS_ADDR IOCTLNUMBER_RWIO(GET_L2_DMA_PHYS_ADDR, struct t_dma_address)
/* #define CP16XX_IOC_OAPP                 IOCTLNUMBER_RWIO(OAPP, struct t_register_app) */
#define CP16XX_IOC_OAPP                 IOCTLNUMBER_NOIO(OAPP /* , struct t_register_app */ ) 
#define CP16XX_IOC_CAPP                 IOCTLNUMBER_NOIO(CAPP /*, struct t_register_app */ )
/* #define CP16XX_IOC_BIND                 IOCTLNUMBER_RWIO(BIND, struct t_read_pool) */
#define CP16XX_IOC_BIND                 IOCTLNUMBER_NOIO(BIND)
#define CP16XX_IOC_UNBIND               IOCTLNUMBER_NOIO(UNBIND)
/* #define CP16XX_IOC_IRTCBF               IOCTLNUMBER_WOIO(IRTCBF, struct t_register_app)*/
#define CP16XX_IOC_IRTCBF               IOCTLNUMBER_NOIO(IRTCBF /* struct t_register_app */ )
#define CP16XX_IOC_SET_DMA_RANGE        IOCTLNUMBER_WOIO(SET_DMA_RANGE, struct t_dma_range)
#define CP16XX_IOC_GET_CP_TIMER         IOCTLNUMBER_ROIO(GET_CP_TIMER, struct t_read_timer)
#define CP16XX_IOCCOUNT                 IOCTLNUMBER_ROIO(COUNT, struct t_read_pool)
#define CP16XX_IOC_PACKET_READ          IOCTLNUMBER_RWIO(PACKET_READ, struct t_dma_pool)
#define CP16XX_IOC_PACKET_WRITE         IOCTLNUMBER_RWIO(PACKET_WRITE, struct t_dma_pool)

#define CP16XX_IOC_OWD                  IOCTLNUMBER_RWIO(OWD, struct t_register_appl_wd)
#define CP16XX_IOC_CWD                  IOCTLNUMBER_WOIO(CWD, struct t_register_appl_wd)
#define CP16XX_IOC_TWD                  IOCTLNUMBER_WOIO(TWD, struct t_register_appl_wd)

/* #define CP16XX_IOC_GET_PNIO_DMA_RANGE   IOCTLNUMBER_RWIO(GET_PNIO_DMA_RANGE, struct t_dma_address) */
#define CP16XX_IOC_GET_PNIO_DMA_RANGE   IOCTLNUMBER_NOIO(GET_PNIO_DMA_RANGE) 

#define CP16XX_IOC_IRTCPID              IOCTLNUMBER_ROIO(IRTCPID, struct t_register_app)
/* #define CP16XX_IOC_MMAP                 IOCTLNUMBER_RWIO(MMAP, struct t_mmap_struct) */
#define CP16XX_IOC_MMAP                 IOCTLNUMBER_NOIO(MMAP)
#define CP16XX_IOC_MUNMAP               IOCTLNUMBER_WOIO(MUNMAP, struct t_mmap_struct)

#endif

#endif /* _CP16XX_H */

