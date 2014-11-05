#ifndef CPS_THREAD_H
#define CPS_THREAD_H

#include "bool.h"
#include "stdbool.h"
#ifdef _WIN32
    #include <windows.h>
    typedef HANDLE CPS_MUTEX_TYPE;
    typedef HANDLE CPS_THREAD_TYPE;
//    typedef DWORD WINAPI CPS_THREAD_RETTYPE;
//    typedef LPVOID CPS_THREAD_PARAMTYPE;
//    typedef LPTHREAD_START_ROUTINE CPS_THREAD_FUNCTYPE;
    #define CPS_THREAD_RETTYPE DWORD WINAPI
    #define CPS_THREAD_ERETTYPE DWORD
    #define CPS_THREAD_PARAMTYPE LPVOID
    #define CPS_THREAD_FUNCTYPE LPTHREAD_START_ROUTINE
    #define CPS_THREAD_INFINITE INFINITE
    #define CPS_THREAD_TIMEOUT -2
    typedef HANDLE CPS_SIG_TYPE;
#else
	#define TRUE true
    #define FALSE false
    // linux stuff goes here
    #include <pthread.h>
    typedef pthread_mutex_t CPS_MUTEX_TYPE;
    typedef pthread_t CPS_THREAD_TYPE;
    typedef struct {
	    pthread_mutex_t lock;
	    pthread_cond_t cond;
	    int count;
    } CPS_SIG_TYPE;
    typedef void* (*CPS_THREAD_FUNCTYPE)(void *);
    typedef void* CPS_THREAD_RETTYPE;
    typedef void* CPS_THREAD_ERETTYPE;
    typedef void* CPS_THREAD_PARAMTYPE;
    #define CPS_THREAD_INFINITE -1;
    #define CPS_THREAD_TIMEOUT -2;
#endif

bool cps_create_mutex(CPS_MUTEX_TYPE *m);
bool cps_lock_mutex(CPS_MUTEX_TYPE *m);
bool cps_unlock_mutex(CPS_MUTEX_TYPE *m);
bool cps_destroy_mutex(CPS_MUTEX_TYPE *m);

CPS_SIG_TYPE *cps_create_sig();
bool cps_lock_sig(CPS_SIG_TYPE *s);
bool cps_unlock_sig(CPS_SIG_TYPE *s);
bool cps_wait_sig(CPS_SIG_TYPE *s);
bool cps_signal_sig(CPS_SIG_TYPE *s);
bool cps_destroy_sig(CPS_SIG_TYPE *s);

bool cps_create_thread(CPS_THREAD_TYPE *t, CPS_THREAD_FUNCTYPE tfFunc, CPS_THREAD_PARAMTYPE tpParam);
int cps_wait_thread(CPS_THREAD_TYPE* t, int timeout);
bool cps_destroy_thread(CPS_THREAD_TYPE *t);
int cps_Sleep(int mSec);

#endif

