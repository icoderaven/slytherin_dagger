#ifndef DEBUG_H
#define DEBUG_H
#include <sys/time.h>
#include <stdio.h>
#define DPFL fprintf(stderr, "%s, %i\n", __FILE__, __LINE__);
#define DP(...) fprintf(stderr, __VA_ARGS__);
#define DPL(L,...) if(debug_h_level > L) fprintf(stderr, __VA_ARGS__);


#ifdef _WIN32
 #include <windows.h>
 void WindowsError(LPTSTR lpszFunction);
#else
 void WindowsError(char * inStr);
 int GetTickCount(void);
#endif

#endif
