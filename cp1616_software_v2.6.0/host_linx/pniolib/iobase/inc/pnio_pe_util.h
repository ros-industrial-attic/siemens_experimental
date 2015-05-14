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
#ifndef _PNI_PE_UTIL_H_
#define _PNI_PE_UTIL_H_


#include "os.h"

#include "pniousrx.h"   //#include "pniobase.h"

//#include "pnio_pe.h"

///////////////////////////////////////////////////////////////////////////////////////
/// Class Mutex is a thin encapsulation of the os specific mutual exclusion API.
/// Mutexes should be embeded in the thread objects and then use through Locks. ->
/// The idea is to represent the acquisition of a mutex as the initialization of a local
/// variable and then rely on the execution of the destructor to release the mutex.
///
class Mutex
{
    friend class Lock;
public:
    Mutex ()
    {
        DPR_MUTEX_CREATE_UNLOCKED(m_mux_obj);
    }

    ~Mutex ()
    {
        DPR_MUTEX_DESTROY(m_mux_obj);
    }
private:
    void Acquire ()
    {
        DPR_MUTEX_LOCK(m_mux_obj);
    }

    void Release ()
    {
        DPR_MUTEX_UNLOCK(m_mux_obj);
    }

    DPR_MUTEX m_mux_obj;
};


///////////////////////////////////////////////////////////////////////////////////////
/// Class Lock simplifies resource management (alloc/free) of mutexes.
/// Lock is a clever object that you construct on the stack and for the duration of its
/// lifetime your object is protected from any other threads.
///
class Lock {
public:
    // Acquire ownership
    Lock ( Mutex& mutex )
    : m_mutex(mutex)
    {
        m_mutex.Acquire();
        TRC_OUT(GR_PE, LV_INFO, "LOCK ");
    }
    // Release ownership
    ~Lock ()
    {
        m_mutex.Release();
        TRC_OUT(GR_PE, LV_INFO, "UN-LOCK ");
    }
private:
    Mutex& m_mutex;
};


#if 0
///////////////////////////////////////////////////////////////////////////////////////
///  interrupt lock object - for fast short access only (VxWorks only)
class  cPeInterruptLock {
    int   flags;
public:
    cPeInterruptLock()  { flags = intLock ();}
    ~cPeInterruptLock() { intUnlock (flags);}
};
#endif // 0


typedef PNIO_UINT8 RqKeyType;

///////////////////////////////////////////////////////////////////////////////////////
/// cPeReq: PE request
///
class cPeReq {

public:
    enum StateEnum {
        st_idle,
        st_wr_pend,   // DR Write Req. is pending
        st_rd_pend,   // DR Read Req. is pending (request for service response)
        st_rd_busy    // DR Read Req. returned ok-busy (we have to retry)
    };

    StateEnum   m_state;
    int         rd_count;
    int         err_count;
    PNIO_UINT32 log_addr;
    PNIO_UINT32 user_handle; // saved user handle from ctr_open()
    PNIO_REF    user_ref;    // saved original ReqRef
    PNIO_UINT8  serv_ref;    // PE req ref: 1-254
    int         lifetime_sec;
    PNIO_PE_CMD_ENUM           cmd_id;
    PNIO_PE_CMD_MODIFIER_ENUM  cmd_modifier;

public:
    cPeReq(PNIO_UINT32 Handle, PNIO_ADDR* pAddr, PNIO_REF user_ref,
           PNIO_PE_CMD_ENUM CmdId, PNIO_PE_CMD_MODIFIER_ENUM CmdModifier, PNIO_UINT8 serv_ref )
      : m_state(st_wr_pend),
        rd_count(0),
        err_count(0),
        log_addr(pAddr->u.Addr),
        user_handle(Handle),
        user_ref(user_ref),
        serv_ref(serv_ref),
        lifetime_sec(PNIO_PE_SERVICE_REQ_LIFETIME_DEFAULT),
        cmd_id(CmdId),
        cmd_modifier(CmdModifier)
    {};

    ~cPeReq() {};

    RqKeyType get_key() {
        return (RqKeyType)serv_ref;
    }

    void set_state(StateEnum st) {
        m_state = st;
    };
};

///////////////////////////////////////////////////////////////////////////////////////
/// cPeRqQueue
///
#include <map>
typedef std::map<RqKeyType, cPeReq*>    RqList;
typedef RqList::iterator                RqListIterator;


class cPeMgt;

class cPeRqQueue {

    Mutex    m_PeReqMutex;
    RqList   m_Rqs;

public:
    cPeRqQueue(cPeMgt& r);    // back reference
    ~cPeRqQueue();

    int     add_rq(cPeReq* rq);
    cPeReq* find_rq(RqKeyType ref);
    cPeReq* remove_rq(RqKeyType ref);
    int     timeout_chk(void);

    cPeMgt& m_cPeMgtBackref;
}; // cPeRqQueue


///////////////////////////////////////////////////////////////////////////////////////
/// cPeServiceRef
///
#include <deque>
typedef std::deque<PNIO_UINT8>   RefList;
typedef RefList::iterator        RefListIterator;


class cPeServiceRef {

    Mutex    m_PeRefMutex;
    RefList  m_Refs;

public:
    cPeServiceRef(const int init_size);
    ~cPeServiceRef();
    PNIO_UINT8 get_ref();
    void       put_ref(PNIO_UINT8 ref);
    int        size();
}; // cPeServiceRef





#endif  // _PNI_PE_UTIL_H_

