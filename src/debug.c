#include "debug.h"

#ifdef _WIN32
 void WindowsError(LPTSTR lpszFunction) { 
    TCHAR szBuf[80]; 
    LPVOID lpMsgBuf;
    DWORD dw = GetLastError(); 

    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM,
        NULL,
        dw,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL );

    wsprintf(szBuf, 
        "%s failed with error %d: %s", 
        lpszFunction, dw, lpMsgBuf); 
 
    //MessageBox(NULL, szBuf, "Error", MB_OK); 

    DP("WindowsError: %s\n", szBuf);
    
    LocalFree(lpMsgBuf);
 }
#else
    #include <sys/time.h>
 void WindowsError(char * inStr) {
	 DP("WindowsError: %s\n", inStr);
 }
 int GetTickCount(void) {
 	struct timeval t;
    long long work;
    gettimeofday(&t,NULL);
    work = t.tv_sec;
    work *= 1000000;
    work += t.tv_usec;
    work /= 1000;
    return (int)work;
 }   
#endif
