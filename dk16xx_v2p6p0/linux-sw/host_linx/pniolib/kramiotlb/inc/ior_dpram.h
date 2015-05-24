/*---------------------------------------------------------------------------
 *  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
 *  Siemens AG. All Rights Reserved.                                         
 *---------------------------------------------------------------------------
 *  Project     : PROFINET IO
 *  Package     : CP1616 DK Firmware
 *  Component   : IO-Router (IOR)
 *  File        : ior_dpram.h
 *  Date        : 12-July-2006
 *
 *---------------------------------------------------------------------------
 *
 *  D e s c r i p t i o n:
 *
 *                Description of DPRAM structures between 
 *                FW (IO-Transfer-Agent) and Host (IO-Concentrator)
 *
 *---------------------------------------------------------------------------
 *
 *  H i s t o r y:
 *                12.07.2006 Dielmann: first implementation
 *
 *--------------------------------------------------------------------------- */

#ifndef _IOR_DPRAM_H
#define _IOR_DPRAM_H

#include "pniobase.h"

#undef ATTR_PACKED
#if defined(_MSC_VER)
 #pragma pack( push, safe_old_packing, 4 )
 #define  ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((aligned (4)))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt ior_dpram.h header for your compiler
#endif


/*-----------------------------------------------------------------*/
/*       IO-Concentrator Table and Shadow Buffers in DPRAM         */
/*-----------------------------------------------------------------*/

/*-----------------------------------------------------------------

  The DPRAM area between IO-Transfer-Agent in Firmware and 
  IO-Concentrator on Host is defined as follows.
  The Header itself starts at offset 0 inside DPRAM area.

  +----------------------------+  <-- All offsets are relative to start of header
  |                            |
  |  IOR_CONCENTRATOR_HEADER   |
  |                            |
  +----------------------------+
  |                            |
  |  IOR_CONCENTRATOR_ENTRY 1  |
  |                            |
  +----------------------------+
  |                            |
  |          ...               |
  |                            |
  +----------------------------+
  |                            |
  |  IOR_CONCENTRATOR_ENTRY n  |
  |                            |
  +----------------------------+
  |  IOR_OUTPUT_SLICE 1        |
  +----------------------------+
  |          ...               |
  +----------------------------+
  |  IOR_OUTPUT_SLICE m        |
  +----------------------------+
  |  ShadowBuffer 1            |
  +----------------------------+
  |  IOXS (Status) 1           |
  +----------------------------+
  |          ...               |
  +----------------------------+
  |  ShadowBuffer n            |
  +----------------------------+
  |  IOXS (Status) n           |
  +----------------------------+

-------------------------------------------------------------------*/

                            
/* Controller output slice (allowed data area for host controller application), 
   complementary to transfer relations */
typedef struct
{
    PNIO_UINT32    ByteLength;          /* Number of bytes, including splittet bytes! */
    PNIO_UINT32    ByteOffset;          /* Byte offset in controller buffer */
    PNIO_UINT32    FirstBitOffset;      /* Bit offset in first byte of controller buffer, values 0...7 */
    PNIO_UINT32    LastBitOffset;       /* Bit offset in last byte of controller buffer, values 0...7 */

} ATTR_PACKED IOR_OUTPUT_SLICE;

/* Concentrator Table Entry: */
/* Remark: NumOfSlices may be 0. This means, that it is not allowed to host application
   to write to this output submodule. This submodule is reserved for transfer only.
   The concentrator will block all host writes to this submodule. 
   In this case there will be no Shadow Buffer (ShadowDataOff=0) and no status buffer (StatusDataOff=0). */
typedef struct
{
    PNIO_UINT32    LogAddr;             /* Logical address, i.e. unique identifier of Controller Submodule */

    PNIO_UINT32    ShadowDataOff;       /* Offset of shadow buffer in DPRAM, may be 0 */
    PNIO_UINT32    ShadowLength;        /* Total Number of bytes in shadow buffer, may be 0 */

    PNIO_UINT32    StatusDataOff;       /* Offset of status buffer in DPRAM, may be 0. Status values PNIO_S_GOOD or PNIO_S_BAD */
    PNIO_UINT32    StatusLength;        /* Total Number of bytes of status buffer, may be 0 */

    PNIO_UINT32    HostStatusDataOff;   /* Offset of host status buffer in DPRAM, may be 0. Status values PNIO_S_GOOD or PNIO_S_BAD */
    PNIO_UINT32    HostStatusLength;    /* Total Number of bytes of host status buffer, may be 0 */

    PNIO_UINT32    WDTime;              /* Watchdog Time for this Controller Submodule, in units of 31.25 us */

    PNIO_UINT32    NumOfSlices;         /* Number of controller output slices for this Controller Submodule, may be 0 */
    PNIO_UINT32    SlicesOff;           /* Offset to array of controller output slices of Type IOR_OUTPUT_SLICE, may be 0 */

} ATTR_PACKED IOR_CONCENTRATOR_ENTRY;

/* Concentrator Table Header: */
typedef struct 
{
    PNIO_UINT32    Validity;            /* Validity Flag for the concentrator table (header, entries, slices), 0=invalid */
    PNIO_UINT32    NumOfConEntries;     /* Current Number of Entries of Type IOR_CONCENTRATOR_ENTRY */
    PNIO_UINT32    TotalNumOfSlices;    /* Current Total Number of all Slices of Type IOR_OUTPUT_SLICE */
    PNIO_UINT32    NewCycleIrqTime;     /* Cycle Time of New Cycle Interrupts, in units of 31.25 us */
    PNIO_UINT32    ConEntriesOff;       /* Offset to array of concentrator entries of Type IOR_CONCENTRATOR_ENTRY */
    PNIO_UINT32    WDTrigger;           /* Watchdog-Trigger. A value not equal 0 starts WD on host, WD will then read WDTime and NewCycleIrqTime */
    PNIO_UINT32    Reserved1;           /* Reserved, set to 0 */
    PNIO_UINT32    Reserved2;           /* Reserved, set to 0 */

} ATTR_PACKED IOR_CONCENTRATOR_HEADER;

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

#endif
