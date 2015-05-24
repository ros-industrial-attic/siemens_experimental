/*-----------------------------------------------------------------------------
 *  Project     : PROFINET IO
 *  Package     : CP1616 DK Firmware
 *  Component   : PROFIenergy ( PE )
 *---------------------------------------------------------------------------
 *
 *  D e s c r i p t i o n:
 *
 *---------------------------------------------------------------------------
 *
 *  H i s t o r y:
 *  12.10.2010 PL: first implementation
 *
 *--------------------------------------------------------------------------- */
#ifndef _PNIO_PE_PDU_H_
#define _PNIO_PE_PDU_H_

#include "os.h"

#undef  ATTR_PACKED
#if defined(_MSC_VER)
 #pragma pack( push, safe_pdu_prev_packing, 1 )
 //#pragma pack(show)
 #define  ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((aligned(1))) __attribute__ ((packed))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt pnioag_rqb.h header for your compiler
#endif


/* Alignment */
typedef struct align_t_ {
    PNIO_UINT8   b;
    PNIO_UINT16  w;
    PNIO_UINT32  i;
} ATTR_PACKED ALIGN_TST;

//#if !defined (offsetof)
//#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
//#endif

#define PE_STATIC_ASSERT(condition) typedef char _static_assert_ [ ((condition) ? 1 : -1) ]



/*-----------------------------------------------------------------------------
 * PE protocol data types:
 *-----------------------------------------------------------------------------
 */

#ifndef PNIO_FLOAT32
#define PNIO_FLOAT32 PNIO_UINT32
#endif

/* Data Structure Identifier:  See Common Application Profile 'PROFIenergy' Order No: 3.802
*/
#define PE_CMD_STRUCT_ID_NOT_AVAILABLE            0  /* No Data Struct present in the PDU */
#define PE_CMD_STRUCT_ID_START_PAUSE_RQ           1  /* RQ: request  */
#define PE_CMD_STRUCT_ID_START_PAUSE_RS           1  /* RS: response */
#define PE_CMD_STRUCT_ID_END_PAUSE_RQ             0
#define PE_CMD_STRUCT_ID_END_PAUSE_RS             1
#define PE_CMD_STRUCT_ID_Q_MODE_LIST_ALL_RQ       0
#define PE_CMD_STRUCT_ID_Q_MODE_LIST_ALL_RS       1
#define PE_CMD_STRUCT_ID_Q_MODE_GET_MODE_RQ       1
#define PE_CMD_STRUCT_ID_Q_MODE_GET_MODE_RS       2
#define PE_CMD_STRUCT_ID_PEM_STATUS_RQ            0
#define PE_CMD_STRUCT_ID_PEM_STATUS_RS            1
#define PE_CMD_STRUCT_ID_PE_IDENTIFY_RQ           0
#define PE_CMD_STRUCT_ID_PE_IDENTIFY_RS           1
#define PE_CMD_STRUCT_ID_Q_MSRMT_GET_LIST_RQ      0
#define PE_CMD_STRUCT_ID_Q_MSRMT_GET_LIST_RS      2
#define PE_CMD_STRUCT_ID_Q_MSRMT_GET_VAL_RQ       1
#define PE_CMD_STRUCT_ID_Q_MSRMT_GET_VAL_RS       1

/* Confirmation Status */
#define PE_CNF_STATE_READY                        1  /* OK */
#define PE_CNF_STATE_READY_ERROR                  2  /* response available with error */
#define PE_CNF_STATE_DATA_INCOMPLETE              3  /* response with partial data    */
#define PE_CNF_STATE_MANUFACT_SPECIFIC         0xD0  /* 0xD0 - 0xFF manufacturer specific */



/* DR_BLOCK_HDR: DataRecord (Read/write) Block Header */
#define PE_DR_BLKTYPE_READ  0x0801
#define PE_DR_BLKTYPE_WRITE 0x0800
#define PE_DR_BLKVERS_HIGT  0x01
#define PE_DR_BLKVERS_LOW   0x00

typedef struct {
    PNIO_UINT16   block_type;    /* 0x0800 RecordDataWrite, 0x0801 RecordDataRead  */
    PNIO_UINT16   block_length;  /* Num bytes without counting the fields BlockType and BlockLength   */
    PNIO_UINT8    version_high;  /* 0x01 */
    PNIO_UINT8    version_low;   /* 0x00 */
} ATTR_PACKED DR_BLOCK_HDR;


/* PE_PDU_HDR: Service Header of Commnd Request & Response */
typedef struct {
    PNIO_UINT8    service_id;    /* service id == CmdId         */
    PNIO_UINT8    rq_ref;        /* service request reference   */
    PNIO_UINT8    modif_state;   /* RQ= cmd modifier, RS= state */
    PNIO_UINT8    struct_id;
} ATTR_PACKED PE_PDU_HDR;

/* PE_PDU_DR_HDR: Pdu header */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;
} ATTR_PACKED PE_PDU_DR_HDR;

/* PE_PDU_DR_READ_RQ: PE DataRecord Read Req buffer  (generic for all 'data record raed rqs')
*/
typedef  PE_PDU_DR_HDR  PE_PDU_DR_READ_RQ;


/* Data of Nagative Confirmation */
typedef struct {
    PNIO_UINT8    err_code;
    PNIO_UINT8    reserved;
} ATTR_PACKED PE_PDU_DT_CONF_NEG;


typedef struct {
    DR_BLOCK_HDR       dr_hdr;
    PE_PDU_HDR         pe_hdr;
    PE_PDU_DT_CONF_NEG pe_neg_cnf;
} ATTR_PACKED PE_PDU_DR_NEG_CNF;


/*-----------------------------------------------------------------------------*/
/*--- START_PAUSE -------------------------------------------------------------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
    PNIO_UINT32   pause_time_ms;     /* requested pause time in ms */

} ATTR_PACKED PE_PDU_START_PAUSE_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT8    pe_mode_id;
    PNIO_UINT8    reserved;
} ATTR_PACKED PE_PDU_START_PAUSE_CONF_POS;

/*
typedef struct {
    union {
        START_PAUSE_CONF_POS  cnf_pos;
        PE_PDU_DT_CONF_NEG    cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_START_PAUSE_CONF;
*/

/*--- END_PAUSE----------------------------------------------------------------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
} ATTR_PACKED PE_PDU_END_PAUSE_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT32   time_to_operate;   /* time to leave energy-saving mode */
} ATTR_PACKED PE_PDU_END_PAUSE_CONF_POS;


/*
typedef struct {
    union {
        END_PAUSE_CONF_POS    cnf_pos;
        PE_PDU_DT_CONF_NEG    cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_END_PAUSE_CONF;
*/

/*--- PE_QUERY_MODES: LIST_ALL    Service_Modifier= 0x01 ----------------------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
} ATTR_PACKED PE_PDU_Q_MODE_LIST_ALL_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT8          num_mode_ids;
    PNIO_UINT8          pe_mode_id[254];     /* array of supported PE_Mode_Ids  MAX=254 */
} ATTR_PACKED PE_PDU_Q_MODE_LIST_ALL_CONF_POS;

/*
typedef struct {
    union {
        Q_MODE_LIST_ALL_CONF_POS cnf_pos;
        PE_PDU_DT_CONF_NEG       cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_Q_MODE_LIST_ALL_CONF;
*/

/*--- PE_QUERY_MODES: GET_MODE    Service_Modifier= 0x02 ----------------------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
    PNIO_UINT8    pe_mode_id;
    PNIO_UINT8    reserved;
} ATTR_PACKED PE_PDU_Q_MODE_GET_MODE_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT8          pe_mode_id;
    PNIO_UINT8          pe_mode_attributes;
    PNIO_UINT32         time_min_pause;
    PNIO_UINT32         time_to_pause;
    PNIO_UINT32         time_to_operate;
    PNIO_UINT32         time_min_length_of_stay;
    PNIO_UINT32         time_max_length_of_stay;
    PNIO_FLOAT32        mode_power_consumption;
    PNIO_FLOAT32        energy_consumption_to_pause;
    PNIO_FLOAT32        energy_consumption_to_operate;
} ATTR_PACKED PE_PDU_Q_MODE_GET_MODE_CONF_POS;

/*
typedef struct {
    union {
        Q_MODE_GET_MODE_CONF_POS cnf_pos;
        PE_PDU_DT_CONF_NEG       cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_Q_MODE_GET_MODE_CONF;
*/

/*--- PEM_STATUS --------------------------------------------------------------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
} ATTR_PACKED PE_PDU_PEM_STATUS_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT8          pe_mode_id_source;
    PNIO_UINT8          pe_mode_id_destination;
    PNIO_UINT32         time_to_operate; /* max time to reach 'operate' state */
    PNIO_UINT32         remaining_time_to_destination;
    PNIO_FLOAT32        mode_power_consumption;
    PNIO_FLOAT32        energy_consumption_to_destination;
    PNIO_FLOAT32        energy_consumption_to_operate;
} ATTR_PACKED PE_PDU_PEM_STATUS_CONF_POS;

/*
typedef struct {
    union {
        PEM_STATUS_CONF_POS  cnf_pos;
        PE_PDU_DT_CONF_NEG   cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_PEM_STATUS_CONF;
*/

/*--- PE_IDENTIFY -------------------------------------------------------------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
} ATTR_PACKED PE_PDU_PE_IDENTIFY_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT8          num_services;    /* number of supported services */
    PNIO_UINT8          service_id[254]; /* array of service ids PNIO_PE_CMD_xxx  */
} ATTR_PACKED PE_PDU_PE_IDENTIFY_CONF_POS;

/*
typedef struct {
    union {
        PE_IDENTIFY_CONF_POS cnf_pos;
        PE_PDU_DT_CONF_NEG   cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_PE_IDENTIFY_CONF;
*/

/*--- PE command Query_Measurement: GET_LIST   Service_Modifier= 0x01 -------
 */
/* REQ:  */
typedef struct {
    DR_BLOCK_HDR  dr_hdr;
    PE_PDU_HDR    pe_hdr;            /* service request header */
} ATTR_PACKED PE_PDU_Q_MSRMT_GET_LIST_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT16         measurement_id;
    PNIO_UINT8          accuracy_domain;
    PNIO_UINT8          accuracy_class;
    PNIO_FLOAT32        range;
} ATTR_PACKED PE_MSRMT_ID_ELEM;

typedef struct {
    PNIO_UINT8          num_measurement_ids;
    PNIO_UINT8          reserved;
    PE_MSRMT_ID_ELEM    elem[PE_MSRMT_IDS_MAX];
} ATTR_PACKED PE_PDU_Q_MSRMT_GET_LIST_CONF_POS;

/*
typedef struct {
    union {
        Q_MSRMT_GET_LIST_CONF_POS cnf_pos;
        PE_PDU_DT_CONF_NEG        cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_Q_MSRMT_GET_LIST_CONF;
*/

/*--- PE command Query_Measurement: GET_VAL    Service_Modifier= 0x02 ---------
 */
/* REQ: */
typedef struct {
    PNIO_UINT8          num_measurement_ids;
    PNIO_UINT8          reserved;
    PNIO_UINT16         measurement_id[PE_MSRMT_IDS_MAX];
} ATTR_PACKED PDU_PRM_Q_MSRMT_GET_VAL_REQ;

typedef struct {
    DR_BLOCK_HDR                     dr_hdr;
    PE_PDU_HDR                       pe_hdr;   /* service request header */
    PDU_PRM_Q_MSRMT_GET_VAL_REQ      prm;
} ATTR_PACKED PE_PDU_Q_MSRMT_GET_VAL_REQ;

/* CONF: */
typedef struct {
    PNIO_UINT16         measurement_id;
    PNIO_UINT16         status_of_measurement_value;
    PNIO_FLOAT32        transmission_data_type;
    PNIO_UINT32         end_of_demand;
    PNIO_UINT32         reserved;
} ATTR_PACKED PE_MSRMT_VAL_ELEM;

typedef struct {
    PNIO_UINT8          num_measurement_values;
    PNIO_UINT8          reserved;
    PE_MSRMT_VAL_ELEM   value[PE_MSRMT_IDS_MAX];
} ATTR_PACKED PE_PDU_Q_MSRMT_GET_VAL_CONF_POS;

/*
typedef struct {
    union {
        Q_MSRMT_GET_VAL_CONF_POS  cnf_pos;
        PE_PDU_DT_CONF_NEG        cnf_neg;
    } dt;
} ATTR_PACKED PE_PDU_Q_MSRMT_GET_VAL_CONF;
*/

typedef struct {
    union {
        PE_PDU_DT_CONF_NEG                 negative_conf;         /* generic neg conf */
        PE_PDU_START_PAUSE_CONF_POS        start_pause_conf;
        PE_PDU_END_PAUSE_CONF_POS          end_pause_conf;
        PE_PDU_Q_MODE_LIST_ALL_CONF_POS    q_mode_list_all_conf;
        PE_PDU_Q_MODE_GET_MODE_CONF_POS    q_mode_get_mode_conf;
        PE_PDU_PEM_STATUS_CONF_POS         pem_status_conf;
        PE_PDU_PE_IDENTIFY_CONF_POS        pe_identify_conf;
        PE_PDU_Q_MSRMT_GET_LIST_CONF_POS   q_msrmt_get_list_conf;
        PE_PDU_Q_MSRMT_GET_VAL_CONF_POS    q_msrmt_get_val_conf;
    } cdt;  /* confirmation data */
} ATTR_PACKED PE_PDU_DT_CONF;

typedef struct {
    DR_BLOCK_HDR       dr_hdr;
    PE_PDU_HDR         pe_hdr;
    PE_PDU_DT_CONF     pe_cnf;
} ATTR_PACKED PE_PDU_CNF;


#if defined(_MSC_VER)
 #pragma pack( pop, safe_pdu_prev_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif


#endif /* _PNIO_PE_PDU_H_  */
