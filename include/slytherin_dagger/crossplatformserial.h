#ifndef CROSSPLATFORMSERIAL_H
#define CROSSPLATFORMSERIAL_H

//#include "bool.h"

#ifdef _WIN32
  #include <windows.h>
  typedef HANDLE FD_TYPE;
  #define CBR_921600 921600
  #define CBR_230400 230400
  #define B921600 CBR_921600
  #define B230400 CBR_230400
  #define B115200 CBR_115200
  #define B57600 CBR_57600
  #define B38400 CBR_38400
  #define B19200 CBR_19200
  #define B9600 CBR_9600
  #define B2400 CBR_2400
#else
  #include <termios.h>
  typedef int FD_TYPE;
#endif

// return value for read timeout
#define CPS_READ_TIMEOUT -2

#define CPS_CLOSED 0
#define CPS_OPENED 1

typedef struct cpsPortStruct{
	FD_TYPE fd;
	int speed;
	int status;
} cpsPort;

typedef cpsPort *pCpsPort;

int cps_OpenPort(pCpsPort port, int baud, char* portname);
int cps_Read(pCpsPort port, char* cpBuf, int iLen, int iTimeout);
int cps_Write(pCpsPort port, char* cpBuf, int iToWrite);
bool cps_ClosePort(pCpsPort port);

#endif // CROSSPLATFORMSERIAL_H
