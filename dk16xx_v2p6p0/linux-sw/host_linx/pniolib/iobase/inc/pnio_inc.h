/*---------------------------------------------------------------------------*/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*  SIMATIC NET. All Rights Reserved.                                        */
/*---------------------------------------------------------------------------*/
/*  Project             : PROFInet IO                                        */
/*  Package             : controller and/or device                           */
/*  Component           : pnio interface                                     */
/*  File                : pnio_inc.h                                         */
/*  Author              : A&D PT2 D, K. Traupe-Seitz                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*  D e s c r i p t i o n :                                                  */
/*  configuration for pnio controller                                        */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#ifndef PNIO_INC_H
#define PNIO_INC_H


/*---------------------------------------------------------------------------*/
/*  MACROS TO ACCESS IOXS in PNIO CSRT frame                                 */
/*---------------------------------------------------------------------------*/

#define PNIO_GET_IOXS_MASK     '\x80'        /* get bit 7 */
#define PNIO_IOXS_BAD          '\x00'       
#define PNIO_IOXS_GOOD         '\x80'       
#define PNIO_IOXS_BYTE_NR      0

/* type define */
#define PNIO_IOXS_DETECT_BY_CONTROLLER   0x60     /* bit 6 und 5 = 1 (detected set-by-controller), bit 0 = 0 */
#define PNIO_IOXS_DETECT_BY_DEVICE       0x40     /* bit 6 = 1 und bit 5 = 0 (means detected-by-device), bit 0 = 0 */
#define PNIO_IOXS_DETECT_BY_SLOT         0x20     /* bit 6 = 0 und 5 = 1 (means detected-by-slot), bit 0 = 0 */
#define PNIO_IOXS_DETECT_BY_SUBSLOT      0x00     /* bit 6 und 5 = 0 (means detected-by-subslot), bit 0 = 0 */


/* macro to set IOXS in frame to good or bad */
/* parameters:
    pIoxs           pointer of ioxs to access
    IoxsLen         len fo ioxs to access, today == 1 (byte)
    val             value to set: true => good, false => bad
    type            value to set: PNIO_IOXS_DETECT_BY_XXXX
*/

#define PNIO_SET_IOXS(pIoxs, IoxsLen, val, type)                                    \
    {                                                                               \
        /* val == true ==> set IOXS to good */                                      \
        if (val)                                                                    \
            *((char *)(pIoxs))              = PNIO_IOXS_GOOD;                       \
        else                                                                        \
            *((char *)(pIoxs))              = PNIO_IOXS_BAD | type;                 \
			if (IoxsLen > 1){\
            unsigned int PNIO_SET_IOXS_count;\
            for(PNIO_SET_IOXS_count = PNIO_IOXS_BYTE_NR+1; PNIO_SET_IOXS_count < (IoxsLen) ; PNIO_SET_IOXS_count++)       \
                ((char *)(pIoxs))[PNIO_SET_IOXS_count]    = 0; /* initialize to 0 */              \
			}																		\
    }                                                                               \


/* macro to get value of IOXS in frame */
/* parameters:
    pIoxs           pointer of ioxs to access
    IoxsLen         len fo ioxs to access, today == 1 (byte)

  use this macro assigning ist to a variable, e.g.
  int ioxsVal;
  ioxsVal = PNIO_GET_IOXS_VALUE(pIoxs, IoxsLen);

*/
#define PNIO_GET_IOXS_VALUE(pIoxs, IoxsLen)                                                       \
    (*((char *)(pIoxs)))

/* macro to get value of IOXS DATA STATE in frame */
/* parameters:
    pIoxs           pointer of ioxs to access
    IoxsLen         len fo ioxs to access, today == 1 (byte)

  use this macro assigning ist to a variable, e.g.
  int ioxsVal;
  ioxsVal = PNIO_GET_IOXS_DATA_STATE(pIoxs, IoxsLen);

  if ioxsVal contains a value != 0 ==> ioxs is good,
  if ioxsVal contains a value == 0 ==> ioxs is bad.
*/

#define PNIO_GET_IOXS_DATA_STATE(pIoxs, IoxsLen)                                                       \
    ((*((char *)(pIoxs))) & PNIO_GET_IOXS_MASK)


#endif /* PNIO_INC_H */

/*---------------------------------------------------------------------------*/
/*  Copyright (C) 2003-2010                                                  */
/*  SIMATIC NET. All Rights Reserved.                                        */
/*---------------------------------------------------------------------------*/
