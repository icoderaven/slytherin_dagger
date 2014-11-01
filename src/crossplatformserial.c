/*! \file crossplatformserial.c
 *  \author Michael Schwerin
 *  \date December 14, 2004
 *  \since 1.0
 *  \brief This provides a linux/windows compatable serial port library
 *
 *  This file contains a cross platform serial port library implementation for
 *  linux and windows NT/2000/XP
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include "debug.h"
#include "crossplatformserial.h"

#ifdef _WIN32
	#include "winserial.h"
#else
	#include "linuxserial.h"
#endif

/*! The cps_OpenPort() function had to be implemented two different ways for 
 *  Windows and Linux.  This function is used to open the serial port which the
 *  device is connected to.
 *  \param port This is the port object representing the serial port to open
 *  \param buad The baudrate to open the serial port at
 *  \param portname Then file/device name of to open (ex. COM1 for windows - /dev/ttyS0 for linux)
 *  \return -1 on error, 0 on success
 */
int cps_OpenPort(pCpsPort port, int baud, char* portname) {
	if(port == NULL) return -1;
	
	#ifdef _WIN32  /* Begin Windows-specific code */
		port->fd = cps_win_Open(baud, portname);
	#else  /* End Windows code, begin Linux-specific code */
		port->fd = cps_lin_Open(baud, portname);
	#endif
	
	port->speed = baud;
	if( (int)port->fd == -1) {
        port->status = CPS_CLOSED;
        return -1;
    } else {
		port->status = CPS_OPENED;
	}
	return 0;
}

/*!
 * Reads from a serial port
 * \param port This is the port object representing the serial port to read from
 * \param cpBuf This is the destination buffer into which new data is stored
 * \param iLen An integer indicating the size of cpBuf or maximum number of bytes that should be read
 * \param iTimeout An integer representing the length of time after which the read should fail and give up (in milliseconds)
 * \return The number of bytes read is returned, or -1 is returned if an error occured
 */
int cps_Read(pCpsPort port, char* cpBuf, int iLen, int iTimeout) {
	if(port == NULL) return -1;
	#ifdef _WIN32
		return cps_win_Read(port->fd, cpBuf, iLen, iTimeout);
	#else
		return cps_lin_Read(port->fd, cpBuf, iLen, iTimeout);
	#endif
}

/*!
 * Writes a buffer to a port
 * \param port This is the port object representing the serial port to write to
 * \param cpBuf A pointer to the character buffer holding the data to be written
 * \param iToWrite An integer indicating the number of bytes which should be written
 * \return -1 on error, otherwise the number of bytes written
 */
int cps_Write(pCpsPort port, char* cpBuf, int iToWrite) {
	if(port == NULL) return -1;
	#ifdef _WIN32
		return cps_win_Write(port->fd, cpBuf, iToWrite);
	#else
		return cps_lin_Write(port->fd, cpBuf, iToWrite);
	#endif
}

/*!
 * Closes the serial port and free any related resources
 * \param port This is the port object representing the serial port to be closed
 * \return TRUE if port is successfully close, otherwise FALSE
 */
bool cps_ClosePort(pCpsPort port) {
	if(port == NULL) return false;
    port->status = CPS_CLOSED;
	#ifdef _WIN32
		if (cps_win_Close(port->fd)) return true;
		else return false;
	#else
		if (cps_lin_Close(port->fd)) return true;
		else return false;
	#endif    
}
