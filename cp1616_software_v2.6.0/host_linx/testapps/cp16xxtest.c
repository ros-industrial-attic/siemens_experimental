/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : cp16xxtest.c
* ---------------------------------------------------------------------------
* DESCRIPTION  : Test application
*****************************************************************************/
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

#include "os.h"
#include "cp16xx.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "traceinfo.h"

#ifdef __cplusplus                       /* If C++ - compiler: Use C linkage */
#define LSA_EXTERN  extern "C"
#else
#define LSA_EXTERN  extern
#endif

#define LSA_CONST_MEM_ATTR                 /* const data attribute */

/* subsystem switched to activ or passive */
typedef enum {
   TRACE_SUBSYS_PASSIVE =  0,
   TRACE_SUBSYS_ACTIV   =  1,
   TRACE_SUBSYS_ACTION_NUM
}LTRC_ACTIVE_PASSIVE_TYPE;

/* intern admin struct for every subsystem */

#define LSA_UINT16                  unsigned short int  /* or unsigned short */
#define LSA_CHAR                    char

typedef  enum {
       TRACE_GROUP_NOTHING  =  0,
       TRACE_GROUP_BOARD    =  1,
       TRACE_GROUP_ICON     =  2,
       TRACE_GROUP_PNIO     =  3,
       TRACE_GROUP_PNIO_CON =  4,
       TRACE_GROUP_CBA      =  5,
       TRACE_GROUP_ETHERNET =  6,
       TRACE_GROUP_PBK      =  7,
       TRACE_GROUP_COMPS    =  8,
       TRACE_GROUP_DPS      =  9,
       TRACE_GROUP_LSA      = 10,
       TRACE_GROUP_LSA_EDD  = 11,
       TRACE_GROUP_LSA_EDDI = 12,
       TRACE_GROUP_LSA_EDDS = 13,
       TRACE_GROUP_LSA_EDDP = 14,
       TRACE_GROUP_H1T      = 15,
       TRACE_GROUP_BACNET   = 16,
       TRACE_GROUP_OMS      = 17,
       TRACE_GROUP_MINIWEB  = 18,
       TRACE_GROUP_CP1628   = 19,
       TRACE_GROUP_DPM      = 20,
       TRACE_GROUP_PBP      = 21,
       TRACE_GROUP_NUM
} LTRC_GROUP_TYPE;

enum ltrc_buffer
{
   TRACE_BUFFER_DEFAULT  =  0, /* dont change the first entry, its necessary for ltrc */

   TRACE_BUFFER_NUM      /* dont change the last entry, its necessary for ltrc  */
};
#define LTRC_BUFFER_TYPE enum ltrc_buffer

enum ltrc_level
{
  TRACE_LEVEL_OFF          = 0,
  TRACE_LEVEL_FATAL        = 1,
  TRACE_LEVEL_ERROR        = 2,
  TRACE_LEVEL_UNEXP        = 3,
  TRACE_LEVEL_WARN         = 4,
  TRACE_LEVEL_NOTE_HIGH    = 5,
  TRACE_LEVEL_NOTE         = 6,
  TRACE_LEVEL_NOTE_LOW     = 7,
  TRACE_LEVEL_CHAT         = 8,

  TRACE_LEVEL_NUM          = 9
};
#define LTRC_LEVEL_TYPE enum ltrc_level

#define LTRC_DEFAULT_RUNTIME_LEVEL     TRACE_LEVEL_ERROR

#define LTRC_TEXT_STRING_LEN           32

typedef  struct {
  LSA_UINT16                    subsysId;
  LSA_UINT16                    instance;
  LSA_CHAR                      name[LTRC_TEXT_STRING_LEN];
  LTRC_ACTIVE_PASSIVE_TYPE      action;
  LTRC_GROUP_TYPE               controllGroup;
  LTRC_BUFFER_TYPE              buffer;
  LTRC_LEVEL_TYPE               level;
}LTRC_ADVTEXT_INFO_TYPE;

#define LTRC_SET_SUBSYS_CFG_TABLE

#include "ltrc_sub.h"



#ifdef RTAI
    #include "rtai.h"
    #include "rtai_sem.h"
    #include "rtai_sched.h"
    #include "rtai_registry.h"
    #include "rtai_lxrt.h"
#endif

#define   SWI_VERSION       0x00019400
#define   SUPPORTED_REVISIONS { 0x02020700, 0x02020800, 0x02020900, 0x02021000, 0x13021000, 0x16021000 }

#define NELEMENTS(array) (sizeof (array) / sizeof ((array) [0]))

#define RET_SUCCESS 0
#define RET_PARSE_ERROR -1
#define RET_DEVICE_ERROR -2
#define RET_OTHER_ERROR -3

/*************** forward declarations ***************/
static int register_read(char *argv[], int argc);
static int register_mmap_and_read(char *argv[], int argc);
static int register_write(char *argv[], int argc);
static int dpram_read(char *argv[], int argc);
static int dpram_mmap_and_read(char *argv[], int argc);
static int dpram_write(char *argv[], int argc);
static int dpram_verify(char *argv[], int argc);
static int dma_read(char *argv[], int argc);
static int dma_mmap_and_read(char *argv[], int argc);
static int dma_write(char *argv[], int argc);
static int interrupt_read(char *argv[], int argc);
static int loop_counter_read(char *argv[], int argc);
static int card_reset(char *argv[], int argc);
static int trace_read(char *argv[], int argc);
static int ertec_dump(char *argv[], int argc);

static int fixme_leak_search(char *argv[], int argc);

#ifdef LATENCY_MEASUREMENT
static int latency(char *argv[], int argc);
#endif

static int write_trace_data(FILE *pfile, TRACE_LINE *tb, unsigned long max_lines, unsigned long w_index);
static int write_trace_config_data(FILE *pfile);
static int set_buffer_freeze(unsigned char freeze);
static int write_exception_buffer(FILE *pfile);

#define REGISTER 0
#define DPRAM 1
#define DMA 2
#define REGISTER_MMAP 10
#define DPRAM_MMAP 11
#define DMA_MMAP 12

static DPR_DRV_HANDLE file;
static int open_control_device(void);
static int close_control_device(void);

static int read_flexible(int kind, unsigned long offset, void *buffer, unsigned long length);
static int write_flexible(int kind, unsigned long offset, void *buffer, unsigned long length);
static int dump(unsigned long offset, unsigned char *buffer, unsigned long length);

typedef struct {
    DPR_UINT32 begin;
    DPR_UINT32 finish;
} PERFTIMES_T;

typedef struct {
    unsigned long total_loops;
    unsigned long current_loop;
    PERFTIMES_T *values;
    char *irte;
} LATENCY_T;

typedef struct {
    const char *cmd;
    int (*f_ptr)(char *argv[], int argc);
    const char *helpText;
    int revChk;
} TOKEN_LIST_T;

typedef struct {
    char key;
    unsigned long val;
    char *help;
} PORT_ATTRIB_T;

/*************** global varables and consts ***************/
static TOKEN_LIST_T CPDrv_TEST_cmdList[] = {
    /* argv, handler, helpText, argc, revChk */
    { "fr", register_read,
        " <start offset[-end offset]> [length] [-raw] [-cp cardindex]\n"
        "\tRead FPGA with ioctl\n"
        "\t-raw - raw dump",
        0 },
    { "fm", register_mmap_and_read,
        " <start offset[-end offset]> [length] [-raw] [-cp cardindex]\n"
        "\tRead FPGA with mmap\n"
        "\t-raw - raw dump",
        0 },
    { "fw", register_write,
        " <start offset> <value> [-cp cardindex]\n"
        "\tWrite FPGA",
        0 },
    { "mr", dpram_read,
        " <start offset[-end offset]> [length] [-raw] [-cp cardindex]\n"
        "\tRead MPRAM with ioctl\n"
        "\t-raw - raw dump",
        0 },
    { "mm", dpram_mmap_and_read,
        " <start offset[-end offset]> [length] [-raw] [-cp cardindex]\n"
        "\tRead MPRAM with mmap\n"
        "\t-raw - raw dump",
        0 },
    { "mw", dpram_write,
        " <start offset> <value> [-cp cardindex]\n"
        "\tWrite MPRAM",
        0 },
    { "mv", dpram_verify,
        " <start offset[-end offset]> <start value> [-cp cardindex]\n"
        "\tVerify MPRAM Range - write, read and compare",
        0 },
    { "dr", dma_read,
        " <start offset[-end offset]> [length] [-raw] [-cp cardindex]\n"
        "\tRead DMA with ioctl\n"
        "\t-raw - raw dump",
        0 },
    { "dm", dma_mmap_and_read,
        " <start offset[-end offset]> [length] [-raw] [-cp cardindex]\n"
        "\tRead DMA with mmap\n"
        "\t-raw - raw dump",
        0 },
    { "dw", dma_write,
        " <start offset> <value> [-cp cardindex]\n"
        "\tWrite DMA",
        0 },
    { "ir", interrupt_read,
        " [-cp cardindex]\n"
        "\tInterruptRead - wait until any RT interrupt arrives",
        0 },
    { "lcr", loop_counter_read,
        " [-cp cardindex]\n"
        "\tLoop counter read - returns current loop value",
        0 },
    { "reset", card_reset,
        " [-cp cardindex] [-y]\n"
        "\tReset cp16xx\n"
        "\t-y - ignore running applications",
        0 },
    { "trace", trace_read,
        " [-cp cardindex] [-f filename]\n"
        "\tRead firmware trace buffer of cp16xx\n"
        "\t-f filename - write trace to file",
        0 },
    { "ertec_dump", ertec_dump,
        "\n"
        "\tCreate ERTEC memory dump, output file ertec_dump.bin\n",
        0 },
#ifdef LATENCY_MEASUREMENT
    { "latency", latency,
        " [-cp cardindex] [-F] [-H] <count of loops>\n"
        "\tmesuare latency, default 10 loops\n"
        "\t-F - make an mesuare thread SCHED_FIFO\n"
        "\t-H - make a histogram, otherwise print data to stdout",
        0 },
#endif
    { "fls", fixme_leak_search,
        "\n"
        "\tfixme\n",
        0 },
};

unsigned long longvalues_in_line = 4;
int raw_dump = 0;
unsigned long card_number = 1;
int ignore_running_apps = 0;
char filename[500] = {0};
int use_fifo = 0, use_histogram = 0;

static int next_non_null_arg(char *argv[], int start, int total)
{
    int k;
    for(k = start; k < total && !argv[k]; k++)
        ;

    if(k >= total) {
        /* no known command found */
        return -1;
    } else {
        return k;
    }
}

// ERTEC intern counter clock
static inline double get_ticks_per_microsecond() { return 100.0; }

static inline float messung(DPR_UINT32 a, DPR_UINT32 b)
{
    if(a > b)
        return (float)((float)b + (~(DPR_UINT32)0) - a);
    else
        return (float)(b - a);
}

/*===========================================================================
* FUNCTION : ParseRWArg
*----------------------------------------------------------------------------
* PURPOSE  : extract arguments for read/write operation from para-
*            meter string
*----------------------------------------------------------------------------
* RETURNS  : return-code: 0 = OK
*                         -1 = ERROR
*----------------------------------------------------------------------------
* INPUTS   : argv = parameter string
*            read  = 0 = write call
*                    1 = read call
* OUTPUTS  : *startAddress = first parameter
*            *range = second parameter or default
*            *value = write value (only by wirte call)
*----------------------------------------------------------------------------
* COMMENTS : parameter must be in right order to guarantee correct function
*==========================================================================*/
static int
ParseRWArg(char *argv[], int argc,
    unsigned long *startAddress, unsigned long *range, DPR_UINT32 *value,
    unsigned long read)
{
    char *pdest;
    char buffer1[200];
    char buffer2[200];
    int result;
    unsigned long endAddress;
    int cmd, first_arg, second_arg;

    if(-1 == (cmd = next_non_null_arg(argv, 1, argc)))
        goto ParseRWArg_fail;

    if(-1 == (first_arg = next_non_null_arg(argv, cmd + 1, argc)))
        goto ParseRWArg_fail;

    second_arg = next_non_null_arg(argv, first_arg + 1, argc);
    /* is it address range ? */
    if((pdest = (char *)strstr(argv[first_arg], "-")) != NULL) {
        /* get start address */
        result = (int)(pdest - argv[first_arg]) + 1;
        memcpy(buffer1, argv[first_arg], result - 1);
        buffer1[result - 1] = 0;
        /* get end address */
        memcpy(buffer2, &argv[first_arg][result], strlen(argv[first_arg]));
        /* copy to buffer */
        sscanf(buffer1, "%X", (int *)startAddress);
        sscanf(buffer2, "%X", (int *)&endAddress);

        /* calculate range */
        *range = endAddress - *startAddress;

        /* range negative ? */
        if(*range < 0) {
            *range = 0;
            goto ParseRWArg_fail;
        } else if(*range < 4) {
            /* min range = 4 */
            *range = 4;
        }

        if(!read) {
            /* get value for write */
            if(-1 == second_arg ||
                (argv[second_arg] == NULL) ||
                (sscanf(argv[second_arg], "%X", (int *)value) != 1))
                goto ParseRWArg_fail;
        }
    } else if(read) { /* read call */
        if(sscanf(argv[first_arg], "%lX", startAddress) == 1) {
            if(-1 == second_arg ||
                (argv[second_arg] == NULL) ||
                (sscanf(argv[second_arg], "%lX", range) != 1)) {
                *range = sizeof(unsigned long); /* default range */
            }
        } else {
            goto ParseRWArg_fail; /* startAddress cannot parsed */
        }
    } else { /* write call */
        *range = sizeof(unsigned long);
        if((sscanf(argv[first_arg], "%lX", startAddress) != 1) ||
            -1 == second_arg ||
            (argv[second_arg] == NULL) ||
            (sscanf(argv[second_arg], "%X", value) != 1))
            goto ParseRWArg_fail;
    }

    return 0;

ParseRWArg_fail:
    fprintf(stderr, "ERROR in ParseRWArg\n");
    return -1;
}

static int open_control_device(void)
{
    char tmp[100];

    snprintf(tmp, sizeof(tmp)-1, DPR_CONTROL_INTERFACE, DRIVER_IDX((DPR_UINT32)card_number));

    if(!(file = DPR_DRV_OPEN(tmp))) {
        fprintf(stderr, "failed to open control device %s\n", tmp);
        return -1;
    }

    return 0;
}

static int close_control_device(void)
{
    DPR_DRV_CLOSE(file);
    return 0;
}

static int fixme_leak_search(char *argv[], int argc)
{
    struct t_register_app app;
    void *mmaped_kramtlb = NULL, *mmaped_ErtecSwiBase = NULL,
        *mmaped_ErtecIOTotal = NULL, *mmaped_IRTDMAImage = NULL;
    int ret;

    while(1) {
        if(open_control_device())
            return -1;

        if(DPR_DRV_IOCTL(file, CP16XX_IOC_OAPP, &app, sizeof(app), sizeof(app))) {
            perror("register app failed");
            return -1;
        }

        mmaped_kramtlb = DPR_DRV_MMAP(file, MMAP_OFFSET_DPRAM + OFFSET_KRAMTLB,
            SIZE_KRAMTLB, app.user_id);
        if(!mmaped_kramtlb) {
            perror("mmaped_kramtlb failed");
            return -1;
        }

        mmaped_ErtecSwiBase = DPR_DRV_MMAP(file, MMAP_OFFSET_IRTE + OFFSET_ERTEC_BASE,
            SIZE_ERTEC_BASE, app.user_id);
        if(!mmaped_ErtecSwiBase) {
            perror("mmaped_ErtecSwiBase failed");
            return -1;
        }

        mmaped_ErtecIOTotal = DPR_DRV_MMAP(file, MMAP_OFFSET_IRTE + OFFSET_IO_TOTAL,
            SIZE_IO_TOTAL, app.user_id);
        if(!mmaped_ErtecIOTotal) {
            perror("mmaped_ErtecIOTotal failed");
            return -1;
        }

        mmaped_IRTDMAImage = DPR_DRV_MMAP(file, MMAP_OFFSET_DMA + OFFSET_DMA_IMAGE,
            SIZE_DMA_IMAGE, app.user_id);
        if(!mmaped_IRTDMAImage) {
            perror("mmaped_IRTDMAImage failed");
            return -1;
        }

        /*----------------------------*/

        ret = DPR_DRV_MUNMAP(file, mmaped_IRTDMAImage,
            SIZE_DMA_IMAGE, app.user_id);
        if(ret)
            perror("unmap mmaped_IRTDMAImage failed");

        ret = DPR_DRV_MUNMAP(file, mmaped_ErtecIOTotal,
            SIZE_IO_TOTAL, app.user_id);
        if(ret)
            perror("unmap mmaped_ErtecIOTotal failed");

        ret = DPR_DRV_MUNMAP(file, mmaped_ErtecSwiBase,
            SIZE_ERTEC_BASE, app.user_id);
        if(ret)
            perror("unmap mmaped_ErtecSwiBase failed");

        ret = DPR_DRV_MUNMAP(file, mmaped_kramtlb,
            SIZE_KRAMTLB, app.user_id);
        if(ret)
            perror("unmap mmaped_kramtlb failed");

        DPR_DRV_IOCTL(file, CP16XX_IOC_CAPP, &app, sizeof(app), 0);

        close_control_device();
    }

    return 0;
}

static int read_flexible(int kind, unsigned long offset, void *buffer, unsigned long length)
{
    struct t_rw_direct data;
    struct t_register_app app;
    void *mmaped;
    int ret, ioctrl;
    unsigned long base_offset;

    switch(kind) {
    case REGISTER:
    case DPRAM:
    case DMA:
        data.offset = offset;
        data.length = length;
        data.data = (unsigned char *)buffer;

        switch(kind){
            case REGISTER: ioctrl = CP16XX_IOCREGR; break;
            case DPRAM: ioctrl = CP16XX_IOCDPRAMR; break;
            case DMA: ioctrl = CP16XX_IOCDMAR; break;
            default: perror("unknown kind"); return -1;
        }

        if(DPR_DRV_IOCTL(file, ioctrl, &data, sizeof(data), sizeof(data))) {
            perror("ioctl failed");
            return -1;
        }
        break;
    case REGISTER_MMAP:
    case DPRAM_MMAP:
    case DMA_MMAP:
        if(DPR_DRV_IOCTL(file, CP16XX_IOC_OAPP, &app, sizeof(app), sizeof(app))) {
            perror("register app failed");
            return -1;
        }

        switch(kind){
            case REGISTER_MMAP: base_offset = MMAP_OFFSET_IRTE; break;
            case DPRAM_MMAP: base_offset = MMAP_OFFSET_DPRAM; break;
            case DMA_MMAP: base_offset = MMAP_OFFSET_DMA; break;
            default: perror("unknown kind"); return -1;
        }

        mmaped = DPR_DRV_MMAP(file, base_offset + offset, length, app.user_id);
        if(!mmaped) {
            perror("mmap failed");
            return -1;
        }

        memcpy(buffer, mmaped, length);

        ret = DPR_DRV_MUNMAP(file, mmaped, length, app.user_id);
        if(ret)
            perror("munmap failed");

        DPR_DRV_IOCTL(file, CP16XX_IOC_CAPP, &app, sizeof(app), 0);

        break;
    };

    return 0;
}

static int write_flexible(int kind, unsigned long offset, void *buffer, unsigned long length)
{
    struct t_rw_direct data;
#if __BYTE_ORDER == __BIG_ENDIAN
    unsigned long i;
    DPR_UINT32 *p;
    p = (DPR_UINT32 *)buffer;
    for(i = 0; i < length/sizeof(DPR_UINT32); i++)
        p[i] = LE_TO_CPU(p[i]);
#endif

    data.offset = offset;
    data.length = length;
    data.data = (unsigned char *)buffer;
    switch(kind) {
    case REGISTER:
    case DPRAM:
        if(DPR_DRV_IOCTL(file, (kind == REGISTER)?CP16XX_IOCREGW:CP16XX_IOCDPRAMW,
                        &data, sizeof(data), 0)) {
            perror("ioctl failed");
            return -1;
        }
        break;
    case DMA:
        if(DPR_DRV_IOCTL(file, CP16XX_IOCDMAW, &data, sizeof(data), 0)) {
            perror("ioctl failed");
            return -1;
        }
        break;
    }

    return 0;
}

static int dump(unsigned long offset, unsigned char *buffer, unsigned long length)
{
    unsigned long i, j, length_in_long;
    DPR_UINT32 *buffer_in_long;
    //char line[256], *p;


    if(raw_dump) {
        for(i = 0; i < length; i++)
            putchar(buffer[i]);
    } else {
        buffer_in_long = (DPR_UINT32*)buffer;
        length_in_long = length / sizeof(DPR_UINT32);

        for(i = 0, j = 0;
            i < length_in_long;
            i++, j++) {
            if(!(j%longvalues_in_line)) {
                /* print address */
                printf("0x%08lX :", offset + (i * sizeof(DPR_UINT32)));
            }
            printf(" 0x%08X", (DPR_UINT32)LE_TO_CPU(buffer_in_long[i]));

            if(!((j+1)%longvalues_in_line)) {
                printf("\n");
            }
        }
        printf("\n");
    }

    return 0;
}



/*===========================================================================
* FUNCTION : dpram_verify
*----------------------------------------------------------------------------
* PURPOSE  : verify write vs read (compare values mpram)
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : argv = parameter string
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int dpram_verify(char *argv[], int argc)
{
    unsigned long startoffset, curoffset;
    unsigned long value = 0, startvalue, expectedvalue;
    unsigned long range;
    unsigned int i;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    range++;                    /* to consider end address to range */
    curoffset = startoffset;
    startvalue = value;

    for(i = 0; i < (range / 4); i++) {
        write_flexible(DPRAM, curoffset, &value, sizeof(value));

        curoffset += sizeof (curoffset);
        value++;
    }

    curoffset = startoffset;
    expectedvalue = startvalue;

    /* read & compare values */
    for(i = 0; i < (range / 4); i++) {
        read_flexible(DPRAM, curoffset, &value, sizeof(value));

        if(value != expectedvalue) {
            printf(
                "ERROR... Read value not match at address %08lX - exp %08lX : rcvd %08lX\n",
                curoffset, expectedvalue, value);
        }

        expectedvalue++;
        curoffset += sizeof (curoffset);
    }

    /* display last address and value got */
    curoffset -= sizeof (curoffset);
    expectedvalue--;
    printf("success! last read - %08lX : %08lX\n", curoffset, expectedvalue);

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : register_read
*----------------------------------------------------------------------------
* PURPOSE  : read from switch register or kram
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : argv = parameter string
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int register_read(char *argv[], int argc)
{
    unsigned long startoffset;
    unsigned long range;
    unsigned char *data;
    int ret;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    data = (unsigned char *)malloc(range);
    if(data) {
        if(!(ret = read_flexible(REGISTER, startoffset, data, range)))
            dump(startoffset, data, range);

        free(data);
    } else {
        ret = -1;
    }

    close_control_device();

    return RET_SUCCESS;
}

static int register_mmap_and_read(char *argv[], int argc)
{
    unsigned long startoffset;
    unsigned long range;
    unsigned char *data;
    int ret;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    data = (unsigned char *)malloc(range);
    if(data) {
        if(!(ret = read_flexible(REGISTER_MMAP, startoffset, data, range)))
            dump(startoffset, data, range);

        free(data);
    } else {
        ret = -1;
    }

    close_control_device();

    return RET_SUCCESS;
}

static int register_write(char *argv[], int argc)
{
    unsigned long startoffset;
    DPR_UINT32 value;
    unsigned long range;

    if(ParseRWArg(argv, argc, &startoffset, &range, &value, 0))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    write_flexible(REGISTER, startoffset, &value, sizeof(value));

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : dpram_read
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : argv = parameter string
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int dpram_read(char *argv[], int argc)
{
    unsigned long startoffset;
    unsigned long range;
    unsigned char *data;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    data = (unsigned char *)malloc(range);
    if(data) {
        if(!read_flexible(DPRAM, startoffset, data, range))
            dump(startoffset, data, range);

        free(data);
    } else {
        fprintf(stderr, "malloc failed\n");
    }

    close_control_device();

    return RET_SUCCESS;
}

static int dpram_mmap_and_read(char *argv[], int argc)
{
    unsigned long startoffset;
    unsigned long range;
    unsigned char *data;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    data = (unsigned char *)malloc(range);
    if(data) {
        if(!read_flexible(DPRAM_MMAP, startoffset, data, range))
            dump(startoffset, data, range);

        free(data);
    } else {
        fprintf(stderr, "malloc failed\n");
    }

    close_control_device();

    return RET_SUCCESS;
}

static int dpram_write(char *argv[], int argc)
{
    unsigned long startoffset;
    DPR_UINT32 value;
    unsigned long range;

    if(ParseRWArg(argv, argc, &startoffset, &range, &value, 0))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    write_flexible(DPRAM, startoffset, &value, sizeof(value));

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : dma_read
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : argv = parameter string
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int dma_read(char *argv[], int argc)
{
    unsigned long startoffset;
    unsigned long range;
    unsigned char *data;
    int ret;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    data = (unsigned char *)malloc(range);
    if(data) {
        if(!(ret = read_flexible(DMA, startoffset, data, range)))
            dump(startoffset, data, range);

        free(data);
    } else {
        ret = -1;
    }

    close_control_device();

    return RET_SUCCESS;
}

static int dma_mmap_and_read(char *argv[], int argc)
{
    unsigned long startoffset;
    unsigned long range;
    unsigned char *data;
    int ret;

    if(ParseRWArg(argv, argc, &startoffset, &range, NULL, 1))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    data = (unsigned char *)malloc(range);
    if(data) {
        if(!(ret = read_flexible(DMA_MMAP, startoffset, data, range)))
            dump(startoffset, data, range);

        free(data);
    } else {
        ret = -1;
    }

    close_control_device();

    return RET_SUCCESS;
}

static int dma_write(char *argv[], int argc)
{
    unsigned long startoffset;
    DPR_UINT32 value;
    unsigned long range;

    if(ParseRWArg(argv, argc, &startoffset, &range, &value, 0))
        return RET_PARSE_ERROR;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    write_flexible(DMA, startoffset, &value, sizeof(value));

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : interrupt_read
*----------------------------------------------------------------------------
* PURPOSE  : Interrupt read function
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int interrupt_read(char *argv[], int argc)
{
    if(open_control_device())
        return RET_DEVICE_ERROR;

    printf("sleep until any RT interrupt arrives ...\n");

    if(DPR_DRV_IOCTL(file, CP16XX_IOCWIRQ, NULL, 0, 0)) {
        perror("ioctl failed");
    }

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : loop_counter_read
*----------------------------------------------------------------------------
* PURPOSE  : Interrupt read function
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int loop_counter_read(char *argv[], int argc)
{
    int ret;
        struct t_read_pool trp;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    ret = DPR_DRV_IOCTL(file, CP16XX_IOCCOUNT, &trp, 0, sizeof(trp));
    printf("get current loop counter ... (%d) %lu\n",
        ret, trp.user_id);

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : card_reset
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int card_reset(char *argv[], int argc)
{
    if(open_control_device())
        return RET_DEVICE_ERROR;

    printf("reset cp %lu ...\n", card_number);

    if(DPR_DRV_IOCTL(file,
        ignore_running_apps ? CP16XX_IOCRESET : CP16XX_IOCSHUTDOWN, NULL, 0, 0)) {
        perror("ioctl failed");
    }

    close_control_device();

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : ShowRevision
*----------------------------------------------------------------------------
* PURPOSE  : show revision number of FPGA
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static void ShowRevision(void)
{
    unsigned long rev;
    if(open_control_device())
        return;

    if(!read_flexible(REGISTER, SWI_VERSION, &rev, 1))
        printf("FPGA Revision: 0x%08lX\n", rev);
    close_control_device();
}

/*===========================================================================
* FUNCTION : CheckRevision
*----------------------------------------------------------------------------
* PURPOSE  : test if revison is supported
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int CheckRevision(void)
{
    unsigned long rev, revTable[] = SUPPORTED_REVISIONS;
    unsigned int i;
    int ret = 0;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    if(!read_flexible(REGISTER, SWI_VERSION, &rev, 1)) {
        for(i = 0; i < NELEMENTS(revTable); i++) {
            if(rev == revTable[i]) {
                ret = 1;
                break;
            }
        }
    }

    close_control_device();
    return ret;
}

/*===========================================================================
* FUNCTION : ShowHelp
*----------------------------------------------------------------------------
* PURPOSE  : show help text
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static void ShowHelp(void)
{
    unsigned int i;

    printf("Use:\n");
    printf("cp16xxtest <");
    for(i = 0; i < NELEMENTS(CPDrv_TEST_cmdList); i++) {
        printf("%s|", CPDrv_TEST_cmdList[i].cmd);
    }
    printf("\b>[-s] [-v]\n");
    for(i = 0; i < NELEMENTS(CPDrv_TEST_cmdList); i++) {
        printf("%s%s\n", CPDrv_TEST_cmdList[i].cmd, CPDrv_TEST_cmdList[i].helpText);
    }
}


/*===========================================================================
* FUNCTION : write_trace_data
*----------------------------------------------------------------------------
* PURPOSE  : Write trace line to trace output file
*----------------------------------------------------------------------------
* RETURNS  : RET_SUCVCESS
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int write_trace_data(FILE *pfile, TRACE_LINE *tb, unsigned long max_lines, unsigned long w_index)
{
    unsigned long index = w_index;

    if(index < max_lines)
        do {
            if(tb[index][0] != 0)
                fprintf(pfile, "TRC_LINE: 0x%08x 0x%08x 0x%08x 0x%08x\n",
                        LE_TO_CPU(tb[index][0]), LE_TO_CPU(tb[index][1]), 
                        LE_TO_CPU(tb[index][2]), LE_TO_CPU(tb[index][3]));

            if(++index == max_lines)
                index = 0;
        } while (index != w_index);

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : write_trace_config_data
*----------------------------------------------------------------------------
* PURPOSE  : Write trace configuration to trace output file
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int write_trace_config_data(FILE *pfile)
{
    int i;

    /* write subsystems to outfile */
    for (i = 0; ltrc_subsys_cfg_table[i].instance != 0xffff; ++i) {
        fprintf(pfile, "TRC_SUBSYS: %05lu %03lu %s\n",
            LE_TO_CPU(ltrc_subsys_cfg_table[i].subsysId),
            LE_TO_CPU(ltrc_subsys_cfg_table[i].instance),
            ltrc_subsys_cfg_table[i].name);
    }

    /* write trace level names to outfile */
    fprintf(pfile, "TRC_LEVEL: 00000 000 OFF\n");
    fprintf(pfile, "TRC_LEVEL: 00001 000 FATAL\n");
    fprintf(pfile, "TRC_LEVEL: 00002 000 ERROR\n");
    fprintf(pfile, "TRC_LEVEL: 00003 000 UNEXPECTED\n");
    fprintf(pfile, "TRC_LEVEL: 00004 000 WARNING\n");
    fprintf(pfile, "TRC_LEVEL: 00005 000 NOTE HIGH\n");
    fprintf(pfile, "TRC_LEVEL: 00006 000 NOTE\n");
    fprintf(pfile, "TRC_LEVEL: 00007 000 NOTE LOW\n");
    fprintf(pfile, "TRC_LEVEL: 00008 000 CHAT\n");

    /* write trace mode names to outfile */
    fprintf(pfile, "TRC_MODE: 00000 000 NOTHING\n");
    fprintf(pfile, "TRC_MODE: 00001 000 BINARY\n");
    fprintf(pfile, "TRC_MODE: 00002 000 TEXT\n");

    /* write trace output channel names to outfile */
    fprintf(pfile, "TRC_OUTCHL: 00000 000 NOTHING\n");
    fprintf(pfile, "TRC_OUTCHL: 00001 000 BINARY\n");
    fprintf(pfile, "TRC_OUTCHL: 00002 000 CONSOLE\n");

    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : set_buffer_freeze
*----------------------------------------------------------------------------
* PURPOSE  : Freeze/Unfreeze the trace buffer of the CP
*----------------------------------------------------------------------------
* RETURNS  : 0 if not freezed, 1 if freezed, or RET_OTHER_ERROR
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int set_buffer_freeze(unsigned char freeze)
{
    unsigned char is_freezed = !!freeze;

    if(!read_flexible(DPRAM,
        TRACE_START + TRACE_ADMIN_OFFSET +
        ((char *)&(((LTRC_BUFFER_ADMIN_TYPE*)0)->is_freezed) - (char *)0),
        &is_freezed, sizeof(is_freezed))) {

        int ret = is_freezed;

        if(!write_flexible(DPRAM,
            TRACE_START + TRACE_ADMIN_OFFSET +
            ((char *)&(((LTRC_BUFFER_ADMIN_TYPE*)0)->is_freezed) - (char *)0),
            &is_freezed, sizeof(is_freezed)))
            return ret;
    }

    fprintf(stderr, "set_buffer_freeze(%d) failed (%s)\n", freeze, DPR_STRERROR());
    return RET_OTHER_ERROR;
}


/*===========================================================================
* FUNCTION : write_exception_buffer
*----------------------------------------------------------------------------
* PURPOSE  : Write content of exception buffer to trace output file
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int write_exception_buffer(FILE *pfile)
{
    /* get exception buffer */
    unsigned char *data = (unsigned char *)malloc(EXCEPTION_BUFFER_LENGTH);
    if(data) {
        if(!read_flexible(DPRAM, EXCEPTION_BUFFER_START, data, EXCEPTION_BUFFER_LENGTH)) {
            /* write trace level names to outfile */
            fprintf(pfile, "%s\n", (char *)data);
        } else {
            fprintf(stderr, "write_exception_buffer failed (%s)\n", DPR_STRERROR());
            free(data);
            close_control_device();

            return RET_DEVICE_ERROR;
        }
        free(data);
    } else {
        fprintf(stderr, "malloc(%lu) failed\n", EXCEPTION_BUFFER_LENGTH);
        close_control_device();

        return RET_OTHER_ERROR;
    }
    return RET_SUCCESS;
}

/*===========================================================================
* FUNCTION : trace_read
*----------------------------------------------------------------------------
* PURPOSE  : Read trace buffer if CP and write its content to the trace output file
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int trace_read(char *argv[], int argc)
{
    FILE *outfile = stdout;
    void *data = NULL;
    int ret = RET_SUCCESS;
    int freeze;
    TRACE_LINE *trace_buffer;
    LTRC_BUFFER_ADMIN_TYPE *buffer_admin;
    unsigned long max_lines;
    unsigned long w_index;
    DPR_UINT32 firmware_version, driver_version;

    if(open_control_device())
        return RET_DEVICE_ERROR;

    fprintf(stderr, "read trace buffer of cp %lu to \"%s\"...\n", card_number,
            filename[0] ? filename : "stdout");

    /* open outfile */
    if (filename[0]) {
        outfile = fopen(filename, "wb");
        if (outfile == 0) {
            fprintf(stderr, "fopen(\"%s\", \"w\") failed (%s)!\n", filename, DPR_STRERROR());

            ret = RET_OTHER_ERROR;
            goto trace_read_fail_openfile;
        }
    } else {
        outfile = stdout;
    }


    /* get trace buffer */
    data = malloc(TRACE_LENGTH);
    if(!data) {
        fprintf(stderr, "malloc(%lu) failed\n", TRACE_LENGTH);

        ret = RET_OTHER_ERROR;
        goto trace_read_fail_malloc;
    }

    freeze = set_buffer_freeze(1);
    if(freeze < 0) {
        ret = RET_DEVICE_ERROR;
        goto trace_read_fail_freeze;
    }

    /* read firmware version */
    ret = read_flexible(DPRAM, 0x100000 /* DPR_CONFIG_AREA_OFFSET */, data, 20);
    if(ret) {
        fprintf(stderr, "trace_read failed (%s)\n", DPR_STRERROR());

        ret = RET_DEVICE_ERROR;
        goto trace_read_fail;
    }

    driver_version = LE_TO_CPU(*(DPR_VUINT32 *)((char*)data + sizeof(DPR_VUINT32)*3)); /* offset of DPR_BASE::RemoteVersion */
    firmware_version = LE_TO_CPU(*(DPR_VUINT32 *)((char*)data + sizeof(DPR_VUINT32)*4)); /* offset of DPR_BASE::LocalVersion */

    ret = read_flexible(DPRAM, TRACE_START, data, TRACE_LENGTH);
    if(ret) {
        fprintf(stderr, "trace_read failed (%s)\n", DPR_STRERROR());

        ret = RET_DEVICE_ERROR;
        goto trace_read_fail;
    }

    /* set trace_buffer, max_lines, w_index */
    trace_buffer = (TRACE_LINE *)data;
    buffer_admin = (LTRC_BUFFER_ADMIN_TYPE *)((char *)data + TRACE_ADMIN_OFFSET);

    max_lines = LE_TO_CPU(buffer_admin->line_number);
    w_index   = LE_TO_CPU(buffer_admin->wr_pos);

    /* write firmware version */
    fprintf(outfile, "CP1616/04 Firmware version: V%d.%d.%d.%d\n",
        (char)(firmware_version>>24) & 0xff, (char)(firmware_version>>16) & 0xff,
        (char)(firmware_version>>8) & 0xff, (char)firmware_version & 0xff);
    /* write driver version */
    fprintf(outfile, "CP1616/04 Driver version: V%d.%d.%d.%d\n",
        (char)(driver_version>>24) & 0xff, (char)(driver_version>>16) & 0xff,
        (char)(driver_version>>8) & 0xff, (char)driver_version & 0xff);
    /* write trace configuration data to outfile */
    write_trace_config_data(outfile);

    /* write trace lines to outfile */
    write_trace_data(outfile, trace_buffer, max_lines, w_index);

    write_exception_buffer(outfile);

trace_read_fail:
    if(freeze == 0)
        set_buffer_freeze(freeze);

trace_read_fail_freeze:
    free(data);

trace_read_fail_malloc:
    if (outfile != stdout)
        fclose(outfile);

trace_read_fail_openfile:
    close_control_device();

    return ret;
}

/*===========================================================================
* FUNCTION : ertec_dump
*----------------------------------------------------------------------------
* PURPOSE  : create ERTEC dump, output file ertec_dump.bin
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int ertec_dump(char *argv[], int argc)
{
    const unsigned long DUMP_LENGTH = 0x200000;
    FILE *outfile = stdout;
        void *data = NULL;
        size_t len = 0;
    strcpy(filename, "ertec_dump.bin");

    if(open_control_device())
        return RET_DEVICE_ERROR;

    fprintf(stderr, "create ERTEC dump of cp %lu to \"%s\"...\n", card_number,
            filename[0] ? filename : "stdout");

    /* open outfile */
    if (filename[0]) {
        outfile = fopen(filename, "wb");
        if (outfile == 0) {
            fprintf(stderr, "fopen(\"%s\", \"w\") failed (%s)!\n", filename, DPR_STRERROR());
            close_control_device();

            return RET_OTHER_ERROR;
        }
    } else {
            printf("error: outputfile not defined\n");
            goto trace_read_fail_openfile;
    }

    /* get trace buffer */
    data = malloc(DUMP_LENGTH);
    if(data) {
        if(! read_flexible(REGISTER, 0, data, DUMP_LENGTH)) {
            len = fwrite(data, 1, DUMP_LENGTH, outfile);

           if(len != DUMP_LENGTH)
            fprintf(stderr, "fwrite(%Zu) failed\n", len);

           free(data);
        }
    } else {
        fprintf(stderr, "malloc(%lu) failed\n", DUMP_LENGTH);
        close_control_device();

        return RET_OTHER_ERROR;
    }

    if (outfile != stdout)
        fclose(outfile);

trace_read_fail_openfile:
    close_control_device();

    return RET_SUCCESS;
}

#ifdef LATENCY_MEASUREMENT
#define MAX_X 20 /* signs in row */
int histogram(char *msg, PERFTIMES_T values[], unsigned long nvalues, unsigned long classes)
{
    double current;
    double minv  = 0.0;
    double maxv  = 0.0;
    double avgv  = 0.0;
    double sumv  = 0.0;
    double sqsum = 0.0;
    double sdev  = 0.0;

    if (nvalues < 1)
        return 2;

    // calculate mimimum, maximum, average and standard deviation
    // of first nvalues in array values
    unsigned long i, j;
    for (i = 1; i < nvalues; ++i) {
        current = messung(values[i].begin, values[i].finish);
        sumv   += (double)current;
        sqsum  += (double)current * (double)current;
        if (minv > 0.0)
            minv = ((double)current > minv)?minv:(double)current;
        else
            minv = (double)current;

        if (maxv > 0.0)
            maxv = ((double)current > maxv)?(double)current:maxv;
        else
            maxv = (double)current;
    }

    avgv  = sumv / nvalues;
    sdev = sqrt((sqsum - ((sumv * sumv) / nvalues)) / nvalues);

    // get counter clock frequency
    double mhz = get_ticks_per_microsecond();
    if (mhz < 0)
        return 3;

    printf("%s\n", msg);
    printf("evaluated %lu values at counter clock frequency: %8.3lf MHz\n", nvalues, mhz);
    printf("minimum: %12.3lf us\n", minv / mhz);
    printf("maximum: %12.3lf us\n", maxv / mhz);
    printf("average: %12.3lf us\n", avgv / mhz);
    printf("std.dev: %12.3lf us\n", sdev / mhz);

    if (classes) {
        double delta       = (maxv - minv) / classes;
        double class_limit = minv;
        unsigned long  max_count   = 0;
        unsigned long  *count       = (unsigned long *)malloc(classes * sizeof(unsigned long));

        if (!count) {
            fprintf(stderr, "malloc(%Zu) failed\n", classes * sizeof(unsigned long));
            return 4;
        } else {
            memset(count, 0, classes * sizeof(unsigned long));
        }

        for (i = 1; i < nvalues; ++i) {
            current = messung(values[i].begin, values[i].finish);
            current -= minv;
            j = (unsigned long)(current / delta);
            if(j >= classes)
                j = classes - 1;
            ++count[j];
        }

        for (i = 0; i < classes; ++i) {
            if (max_count < count[i])
                max_count = count[i];
        }

        printf("\ndistribution of %lu values(first value ommited) in %lu classes:\n", nvalues-1, classes);
        printf("             range                values\n");

        for (i = 0, class_limit = minv; i < classes; ++i, class_limit += delta) {
            unsigned long cnt = count[i];
            if (max_count > MAX_X)
                cnt = (((cnt * MAX_X) + (max_count / 2)) / max_count);

            printf("%c%12.3lf .. %12.3lf] us: %lu: ",
                    (i == 0) ? '[' : '(',
                    class_limit / mhz,
                    ((i == classes - 1) ? maxv : (class_limit + delta)) / mhz,
                    count[i]);

            for (j = 0; j < cnt; ++j)
                printf("%c", '#');
            printf("\n");
        }

        free(count);
    }

    return 0;
}

#ifdef RTAI
void* ptFunc(void *arg)
{
    SEM* sem;
    RT_TASK *rttask;
    unsigned long rttask_name = nam2num("cln");
    struct sched_param mysched;
    LATENCY_T *lm = (LATENCY_T *)arg;
    char tmp[200], *ptmp;

    mysched.sched_priority = 99;
    if(sched_setscheduler(0, SCHED_FIFO, &mysched) == -1) {
        printf("ERROR in settings SCHED_FIFO");
        perror("errno");
        exit(0);
    }

    //mlockall(MCL_CURRENT | MCL_FUTURE);

    if(!(rttask = rt_task_init(rttask_name, 0, 0, 0))) {
        printf("cannot init client task\n");
        exit(1);
    }

    ptmp = RTAI_UNIC_NAME(tmp, RTAI_MAGIC_STARTOP, (int)card_number);
    if(!(sem = (SEM *)rt_get_adr(nam2num(ptmp)))) {
        printf("cannot get addr of sema\n");
        exit(2);
    }
    printf("thread runs RTAI variant, RT semaphore at %p\n", sem);

    rt_make_hard_real_time();
    do {
        if(SEM_ERR == rt_sem_wait(sem)) {
            printf("error by waiting on semaphore\n");
            exit(-1);
        }
        *(DPR_UINT32 *)(lm->irte + 0x13400) |= 0x03;
        lm->values[lm->current_loop].begin  = *(DPR_UINT32 *)(lm->irte + 0x11418);
        lm->values[lm->current_loop].finish = *(DPR_UINT32 *)(lm->irte + 0x11414);
        lm->current_loop++;
    } while(lm->current_loop < lm->total_loops);
    rt_make_soft_real_time();

    rt_task_delete(rttask);

    //munlockall();

    return NULL;
}
#else
void* ptFunc(void *arg)
{
    LATENCY_T *lm = (LATENCY_T *)arg;

    printf("thread runs syscall variant\n");
    do {
        read(fileno(file),  NULL, CP16XX_STARTOP);
        write(fileno(file), NULL, CP16XX_OPDONE);
        lm->values[lm->current_loop].begin  = *(DPR_UINT32 *)(lm->irte + 0x11418);
        lm->values[lm->current_loop].finish  = *(DPR_UINT32 *)(lm->irte + 0x11414);
        lm->current_loop++;
    } while(lm->current_loop < lm->total_loops);

    return NULL;
}
#endif

/*===========================================================================
* FUNCTION : latency
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int latency(char *argv[], int argc)
{
    int ret = RET_OTHER_ERROR, cmd;
    LATENCY_T lm;
    pthread_attr_t temptAttribute;
    pthread_t tHndl;
    struct t_register_app tmp_app;

    memset(&lm, 0, sizeof(lm));

    if(-1 == (cmd = next_non_null_arg(argv, 2, argc)))
        lm.total_loops = 10;
    else if(sscanf(argv[cmd], "%lu", &lm.total_loops) != 1)
        return RET_PARSE_ERROR;

    lm.values = (PERFTIMES_T *)malloc(lm.total_loops * sizeof(PERFTIMES_T));
    if(!lm.values) {
        fprintf(stderr, "no memory, exit\n");
        goto latency_nomem;
    }

    if(open_control_device()) {
        ret = RET_DEVICE_ERROR;
        goto latency_nodev;
    }

    if(ioctl(fileno(file), CP16XX_IOC_OAPP, &tmp_app)) {
        fprintf(stderr, "open application failed, exit\n");
        ret = RET_OTHER_ERROR;
        goto latency_fail_ioctl;
    }

    tmp_app.flags = REG_STARTOP;
    if(ioctl(fileno(file), CP16XX_IOC_IRTCBF, &tmp_app)) {
        fprintf(stderr, "register startop failed, exit\n");
        ret = RET_OTHER_ERROR;
        goto latency_nommap;
    }

    lm.irte = (char*)mmap(0, 0x00100000, PROT_READ | PROT_WRITE, MAP_SHARED,
        fileno(file), MMAP_OFFSET_IRTE);
    if(MAP_FAILED == lm.irte || !lm.irte) {
        fprintf(stderr, "failed to mmap (%s)\n", DPR_STRERROR());
        goto latency_nommap;
    }

    fprintf(stderr, "start measure with %lu loops\n", lm.total_loops);

    /* create threads*/
    pthread_attr_init(&temptAttribute);
    pthread_attr_setschedpolicy(&temptAttribute, use_fifo?SCHED_FIFO:SCHED_OTHER);
    if ((ret = pthread_create(&tHndl, &temptAttribute, ptFunc, &lm)) != 0) {
        fprintf(stderr, "Thread creation failed %d\n", ret);
        goto latency_nothread;
    }

    while(lm.current_loop < lm.total_loops) {
        sleep(1);
    }
    fprintf(stderr, "stop thread\n");
    pthread_join(tHndl, NULL);

    if(use_histogram) {
        histogram("", lm.values, lm.total_loops, 10);
    } else {
        float mhz = get_ticks_per_microsecond();
        float ftmp;
        unsigned long current_loop_index;
        for(current_loop_index = 0; current_loop_index < lm.total_loops; current_loop_index++) {
            ftmp = messung(lm.values[current_loop_index].begin, lm.values[current_loop_index].finish);
            printf("%f %f\n", (float)current_loop_index, ftmp / mhz);
        }
        printf("\n");
    }
    ret = RET_SUCCESS;

latency_nothread:
    munmap(lm.irte, 0x00100000);

latency_nommap:
    ioctl(fileno(file), CP16XX_IOC_CAPP, &tmp_app);

latency_fail_ioctl:
    close_control_device();

latency_nodev:
    free(lm.values);

latency_nomem:

    return ret;
}
#endif

/*===========================================================================
* FUNCTION : main
*----------------------------------------------------------------------------
* PURPOSE  : main entry for program
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int main(int argc, char *argv[])
{
    unsigned int i;
    int k;

    /* check if there are arguments */
    if(argc == 1) {
        ShowRevision();
        ShowHelp();
        return -1;
    }

    /* first check common arguments */
    for(k = 1; k < argc; k++) {
        if(!strcmp(argv[k], "-raw")) {
            argv[k] = NULL;
            raw_dump = 1;
        } else if(!strcmp(argv[k], "-v")) {
            argv[k] = NULL;
        } else if(!strcmp(argv[k], "-cp")) {
            argv[k] = NULL;
            k++;
            card_number = atol(argv[k]);
            argv[k] = NULL;
        } else if(!strcmp(argv[k], "-y")) {
            ignore_running_apps = 1;
            argv[k] = NULL;
        } else if(!strcmp(argv[k], "-F")) {
            use_fifo = 1;
            argv[k] = NULL;
        } else if(!strcmp(argv[k], "-H")) {
            use_histogram = 1;
            argv[k] = NULL;
        } else if(!strcmp(argv[k], "-f")) {
            argv[k] = NULL;
            k++;
            strncpy(filename, argv[k], sizeof(filename));
            argv[k] = NULL;
        }
    }

    /* search for first non NULL arg,
       it's a cmd */
    k = next_non_null_arg(argv, 1, argc);
    if(k == -1) {
        /* no known command found */
        ShowHelp();
        return -1;
    }

    for(i = 0; i < NELEMENTS(CPDrv_TEST_cmdList); i++) {
        if(!strcmp(argv[k], CPDrv_TEST_cmdList[i].cmd)) {
            break;
        }
    }

    if(i >= NELEMENTS(CPDrv_TEST_cmdList)) {
        k = -1;
        /* no known command found */
    } else {
        if(!CPDrv_TEST_cmdList[i].revChk)
            k = CPDrv_TEST_cmdList[i].f_ptr(argv, argc);
        else if(CheckRevision() > 0)
            k = CPDrv_TEST_cmdList[i].f_ptr(argv, argc);
        else {
            fprintf(stderr, "FPGA revision not supported !\n");
            k = 0;
        }
    }

    if(RET_PARSE_ERROR == k) {
        ShowHelp();
        return -1;
    }

    return 0;
}
