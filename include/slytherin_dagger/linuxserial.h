#ifndef LINUXSERIAL_H
#define LINUXSERIAL_H

int cps_lin_Open(int baud, char* portname);
int cps_lin_Read(int iPort, char* cpBuf, int iLen, int iTimeout);
int cps_lin_Write(int iPort, char* cpBuf, int iToWrite);
bool cps_lin_Close(int iPort);

#define TRUE true
#define FALSE false

#endif
