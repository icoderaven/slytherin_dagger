/*! \file cps_thread.c
 *  \author Michael Schwerin
 *  \date December 14, 2004
 *  \since 1.0
 *  \brief This provides a linux/windows compatable thread/mutex/signal library
 *
 *  This file contains a cross platform thread library implementation for
 *  linux and windows NT/2000/XP
 */
#include <stdio.h>
#include "cps_thread.h"
#include "debug.h"

#ifndef _WIN32
	#include <unistd.h>
	#include <stdlib.h>
#endif

/*! Initializes a mutex object
 *  \param m points to an allocated mutex object to be initialized
 *  \return FALSE on error, TRUE on success
 */
bool cps_create_mutex(CPS_MUTEX_TYPE* m) {
	if( m == NULL ) return FALSE;
    #ifdef _WIN32
        *m = CreateMutex(NULL,FALSE,NULL);
        if(*m == NULL) {
            WindowsError("CreateMutex()");
            return FALSE;
        }
    #else
    	if( pthread_mutex_init(m, NULL) != 0) {
	    	perror("Error initializing mutex");
	    	return FALSE;
    	}
    #endif
    return TRUE;
}

/*! locks a given mutex
 *  \param m points to an allocated and initialized mutex object to be locked
 *  \return FALSE on error, TRUE on success
 */
bool cps_lock_mutex(CPS_MUTEX_TYPE* m) {
	if( m == NULL ) return FALSE;
    #ifdef _WIN32
        DWORD dwResult;
        dwResult = WaitForSingleObject(*m, INFINITE);
        switch(dwResult) {
            case WAIT_OBJECT_0:
                break;
            case WAIT_TIMEOUT:
                return FALSE;   // Should never happen with infinite timeout
            case WAIT_ABANDONED:
                return FALSE;
            default:
            	WindowsError("LockMutex()");
            	return FALSE;
        }
    #else
    	if( pthread_mutex_lock(m) != 0) {
	    	perror("Error locking mutex");
	    	return FALSE;
    	}    
    #endif
    return TRUE;
}

/*! unlocks a given mutex
 *  \param m points to an allocated and initialized mutex object to be unlocked
 *  \return FALSE on error, TRUE on success
 */
bool cps_unlock_mutex(CPS_MUTEX_TYPE* m) {
	if( m == NULL ) return FALSE;
    #ifdef _WIN32
        if (! ReleaseMutex(*m)) {
	        WindowsError("ReleaseMutex()");
            return FALSE;
        }
    #else
        if( pthread_mutex_unlock(m) != 0) {
	    	perror("Error unlocking mutex");
	    	return FALSE;
    	}    
    #endif
    return TRUE;
}

/*! frees the operating system resources related to a mutex (doesn't free the memory pointed to by m)
 *  \param m points to an allocated and initialized mutex object to be freed
 *  \return FALSE on error, TRUE on success
 */
bool cps_destroy_mutex(CPS_MUTEX_TYPE* m) {
	if( m == NULL ) return FALSE;
    #ifdef _WIN32
        if(! CloseHandle(*m) ) {
            WindowsError("cps_destroy_mutex()");
            return FALSE;
        }
    #else
    	if( pthread_mutex_destroy(m) != 0) {
	    	perror("Error destroying mutex");
	    	return FALSE;
    	}
    #endif
    return TRUE;
}

/*! Creates a thread running a given function and that function a single arguement
 *  \param t points to an allocated thread type object which will represent the newly created thread
 *  \param tfFunc is a function pointer for the function which this thread will execute
 *  \param tpParm a single arguement which will be passed to tfFunc when it is called
 *  \return FALSE on error, TRUE on success
 */
bool cps_create_thread(CPS_THREAD_TYPE* t, CPS_THREAD_FUNCTYPE tfFunc, CPS_THREAD_PARAMTYPE tpParam) {
	if( t == NULL ) return FALSE;
    #ifdef _WIN32
        DWORD dwThreadId;
        *t = CreateThread(NULL,
                          0,
                          tfFunc,
                          tpParam,
                          0,
                          &dwThreadId);
        if(*t == NULL) {
            WindowsError("cps_create_thread()");
            return FALSE;
        }
    #else
    	if( pthread_create(t, NULL, tfFunc, tpParam) != 0) {
	    	perror("Error creating thread");
	    	return FALSE;
    	}
    #endif
    return TRUE;
}

/*! Waits for a thread to finish execution
 *  \param t points to an allocated and initialized thread object which will be waited for
 *  \param timeout this is how long to wait before giving up.  Use CPS_THREAD_INFINITE to wait forever
 *  \return -1 on error, 0 on success, CPS_THREAD_TIMEOUT if timeout value exceeded
 */
int cps_wait_thread(CPS_THREAD_TYPE* t, int timeout) {
	if( t == NULL ) return -1;
    #ifdef _WIN32
        DWORD dwResult;
        dwResult = WaitForSingleObject(*t, timeout);
        switch(dwResult) {
            case WAIT_OBJECT_0:
                break;
            case WAIT_TIMEOUT:
                return CPS_THREAD_TIMEOUT;   // Should never happen with infinite timeout
            case WAIT_ABANDONED:
                return -1;
        }
        return 0;        
    #else
    	return -1;
    #endif
}

/*! Forces a thread to stop execution
 *  \param t points to an allocated and initialized thread object which will be stopped
 *  \return FALSE on error, TRUE on success
 */
bool cps_destroy_thread(CPS_THREAD_TYPE* t) {
	if( t == NULL ) return FALSE;
    #ifdef _WIN32
        if (!TerminateThread(*t, 0)) {
            WindowsError("cps_destroy_thread()");
            return FALSE;
        }
    #else

    #endif
   	return TRUE;
}


/*!
 * Pauses program execution for a given length of time
 * \param mSec the number of milliseconds to sleep for
 * \return 0 on success, -1 on error
 */
int cps_Sleep(int mSec) {
	#ifdef _WIN32
		Sleep(mSec);
		return 0;
	#else
		return usleep(mSec*1000);
	#endif
}

/*!
 * Creates a singal object (allocates memory for it) and initializes it
 * \return a pointer to the new signal object on success, NULL on error
 */
CPS_SIG_TYPE *cps_create_sig() {
	CPS_SIG_TYPE *retval = (CPS_SIG_TYPE *)malloc(sizeof(CPS_SIG_TYPE));
	#ifdef _WIN32
		if(retval == NULL) {
			WindowsError("cps_create_sig() malloc failed");
			return NULL;
		}
		*retval = CreateEvent(NULL, FALSE, FALSE, NULL);
		if( *retval == NULL ) {
            WindowsError("cps_create_sig()");
            free(retval);
            return NULL;
        }
	#else
		if(retval == NULL) {
			perror("malloc in cps_create_sig()");
			return NULL;
		}
		if( cps_create_mutex(&retval->lock) == FALSE ) {
			DP("Error creating signal object\n");
			free(retval);
			return NULL;
		}
		if( pthread_cond_init( &retval->cond , NULL) != 0) {
			perror("creating condition in signal object");
			free(retval);
			return NULL;
		}
		retval->count = 0;
	#endif
	return retval;	
}
/*!
 * Performs the pre-wait/pre-signal lock
 * \param s points to the signal object on which to perform this lock
 * \return FALSE on error, TRUE on success
 */
bool cps_lock_sig(CPS_SIG_TYPE *s) {
	if(s == NULL) return FALSE;
	
	#ifdef _WIN32
	
	#else
		if( !cps_lock_mutex(&s->lock) ) {
	    	perror("Error locking signal mutex");
	    	return FALSE;
    	}
	#endif
	
	return TRUE;
}

/*!
 * Performs the pre-wait/pre-signal lock
 * \param s points to the signal object on which to perform this lock
 * \return FALSE on error, TRUE on success
 */
bool cps_unlock_sig(CPS_SIG_TYPE *s) {
	if(s == NULL) return FALSE;
	
	#ifdef _WIN32
	
	#else
		if( !cps_unlock_mutex(&s->lock) ) {
	    	perror("Error unlocking signal mutex");
	    	return FALSE;
    	}		
	#endif
	
	return TRUE;
}
/*!
 * Suspends the calling thread until the signal is raised unless it has already been raised
 * \param s points to the signal object on which to perform this wait
 * \return FALSE on error, TRUE on success
 */
bool cps_wait_sig(CPS_SIG_TYPE *s) {
	if(s == NULL) return FALSE;
	
	#ifdef _WIN32
		if( WaitForSingleObject(*s ,INFINITE) != WAIT_OBJECT_0) {
			WindowsError("cps_wait_sig()");
			return -1;
		}
	#else
        if( s->count > 0 ) {
            s->count = 0;;
            return TRUE;
        }
		if( pthread_cond_wait(&s->cond, &s->lock) != 0) {
			perror("Waiting on condition variable");
			return FALSE;
		}
		s->count = 0;
	#endif
	
	return TRUE;
}
/*!
 * raises the signal on this object which will make one waiting thread resume execution
 * \param s points to the signal object on which to raise the signal
 * \return FALSE on error, TRUE on success
 */
bool cps_signal_sig(CPS_SIG_TYPE *s) {
	if(s == NULL) return -1;
	
	#ifdef _WIN32
		if( SetEvent(*s) == 0 ) {
			WindowsError("cps_signal_sig()");
			return FALSE;
		}
	#else
		if( pthread_cond_signal(&s->cond) != 0) {
			perror("singaling a condition variable");
			return FALSE;
		}
		s->count = 1;
	#endif
	
	return TRUE;
}
/*!
 * releases the operating system resources and frees the memory related to this signal object as
 * set up by cps_create_sig
 * \param s points to the signal object to destroy
 * \return 0 on success, -1 on error
 */
bool cps_destroy_sig(CPS_SIG_TYPE *s) {
	int retval = TRUE;
	if(s == NULL) return FALSE;
	
	#ifdef _WIN32
		if( CloseHandle(*s) == 0 ) {
			WindowsError("cps_destroy_sig()");
			retval = FALSE;
		}
	#else
		if( pthread_cond_destroy( &s->cond ) != 0 ) {
			perror("Destroying signal object condition variable");
			retval = FALSE;
		}
		if( pthread_mutex_destroy( &s->lock ) != 0 ) {
			perror("Destroying signal object lock variable");
			retval = FALSE;
		}
	#endif
	
	free(s);
	return retval;
}


