/*! \file winserial.c
 *  \author Michael Schwerin
 *  \date April 19, 2005
 *  \since 1.0
 *  \brief This provides a linux serial port implementation for the crossplatformserial.c file to use
 *
 *  This file contains the linux specific implementation of the cross platform serial port library
 *  which is glued together by the crossplatformserial.c file
 *  The serial device being connected to is assumed to support RTS/CTS hardware flow control handshake
 */
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <strings.h>
#include <errno.h>
#include "crossplatformserial.h"
#include "linuxserial.h"
#include "debug.h"
 
/*! This function is used to open a serial port for reading and writing.
 *  \param buad The baudrate to open the serial port at
 *  \param portname Then file/device name of to open (ex. COM1 for windows - /dev/ttyS0 for linux)
 *  \return An integer representing the file descriptor of the serial port or -1 on an error
 */
int cps_lin_Open(int baud, char* portname) {
	int fd;
	struct termios tio;
    
//	fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fd = open(portname, O_RDWR | O_NOCTTY );
	if (fd < 0) {
		perror(portname);
		return -1;
	}
	
    DP("Opened serial port file @ %s, baud=%i, fd=%i\n", portname, baud, fd)
//	fcntl(fd, F_SETFL, FNDELAY);
	
	bzero(&tio, sizeof(tio));
	
	tio.c_cflag = baud | CRTSCTS | CS8 | CLOCAL | CREAD;
	tio.c_iflag = IGNBRK | IGNPAR;
	tio.c_oflag = 0;

	/* set input mode (non-canonical, no echo, ...) */
	tio.c_lflag = 0;

	/*
	initialize all control characters
	default values can be found in /usr/include/termios.h, and are given
	in the comments, but we don't need them here
	*/
	tio.c_cc[VINTR]    = 0; 	/* Ctrl-c */
	tio.c_cc[VQUIT]    = 0; 	/* Ctrl-\ */
	tio.c_cc[VERASE]   = 0; 	/* del */
	tio.c_cc[VKILL]    = 0; 	/* @ */
	tio.c_cc[VEOF]     = 4; 	/* Ctrl-d */
	tio.c_cc[VTIME]    = 10; 	/* inter-character timer unused */
	tio.c_cc[VMIN]     = 0; 	/* blocking read until 1 character arrives */
	tio.c_cc[VSWTC]    = 0; 	/* '\0' */
	tio.c_cc[VSTART]   = 0; 	/* Ctrl-q */
	tio.c_cc[VSTOP]    = 0; 	/* Ctrl-s */
	tio.c_cc[VSUSP]    = 0; 	/* Ctrl-z */
	tio.c_cc[VEOL]     = 0; 	/* '\0' */
	tio.c_cc[VREPRINT] = 0; 	/* Ctrl-r */
	tio.c_cc[VDISCARD] = 0; 	/* Ctrl-u */
	tio.c_cc[VWERASE]  = 0; 	/* Ctrl-w */
	tio.c_cc[VLNEXT]   = 0; 	/* Ctrl-v */
	tio.c_cc[VEOL2]    = 0; 	/* '\0' */


	if( tcflush(fd, TCIFLUSH) == -1) {
		perror("tcflush on serial device");
		return -1;
	}
	if( tcsetattr(fd, TCSANOW, &tio) == -1 ) {
		perror("Setting serial device parameters");
		return -1;
	}
    DP("Finished opening serial port and setting mode\n");
	return fd;
}

/*!
 * Closes the serial port and free any related resources
 * \param iPort This is the file descriptor of the serial port to be closed
 * \return TRUE if port is successfully close, otherwise FALSE
 */
bool cps_lin_Close(int iPort) {
	if( close(iPort) == -1) {
		perror("Closing serial port");
		return FALSE;
	}
	return TRUE;
}

// returns a-b in msec;
int timeval_msecdiff(struct timeval *a, struct timeval *b) {
	long long work;
	work = a->tv_sec - b->tv_sec;
	work *= 1000000;
	work += a->tv_usec;
	work -= b->tv_usec;
	work /= 1000;
	return (int)work;
}

/*!
 * Reads from a serial port
 * \param hPort This is the HANDLE of the serial port to read from
 * \param cpBuf This is the destination buffer into which new data is stored
 * \param dwLen An integer indicating the size of cpBuf or maximum number of bytes that should be read
 * \param dwTimeout An integer representing the length of time after which the read should fail and give up (in milliseconds)
 * \return The number of bytes read is returned, or -1 is returned if an error occured
 */
int cps_lin_Read(int iPort, char* cpBuf, int iLen, int iTimeout) {
	struct timeval start, now;
	int len;
	int workOffset;
    
//    DP("Call to cps_read( %i, %p, %i, %i )\n", iPort, cpBuf, iLen, iTimeout);
    
	if( gettimeofday(&start, NULL) == -1 ) {
		perror("Getting time of day");
		return -1;
	}
	
	workOffset = 0;
	do {
		len = read(iPort, cpBuf+workOffset, iLen-workOffset);
		if(len == -1) {
        	if( errno == EAGAIN ) {
            	usleep(1000);
                continue;
            }
			perror("Reading from serial port");
			return -1;
		}
		workOffset += len;
		if( workOffset >= iLen ) {
			// we satisfied the length requirment so exit
			break;
		}
		// not done so sleep a 10ms so we aren't busy waiting
		usleep(1000);
		// update the current time for figuring out if timeout has occured
		if( gettimeofday(&now, NULL) == -1 ) {
			perror("Getting time of day");
			return -1;
		}		
	} while( timeval_msecdiff(&now, &start) > iTimeout );
	
/*     for(len = 0; len < workOffset; len++) {
    	if( cpBuf[len] >= 32 )
        	DP("%c ", cpBuf[len])
        else
        	DP("%02X ", cpBuf[len])
    }
    DP("\n");
    
    DP("Finished cps_read() with workOffset %i\n", workOffset); */
    
	return workOffset;
}



/*!
 * Writes a buffer to a port
 * \param hPort This is the HANDLE of the serial port to write to
 * \param cpBuf A pointer to the character buffer holding the data to be written
 * \param dwToWrite An integer indicating the number of bytes which should be written
 * \return -1 on error, otherwise the number of bytes written
 */
int cps_lin_Write(int iPort, char* cpBuf, int iToWrite) {
	int len;
/*    DP("Call to cps_write(%i, %p, %i)\n", iPort, cpBuf, iToWrite);
    
    for(len = 0; len < iToWrite; len++) {
    	if( cpBuf[len] > 32 )
        	DP("%c ", cpBuf[len])
        else
        	DP("%02X ", cpBuf[len])
    }
    DP("\n");*/
        
	len = write(iPort, cpBuf, iToWrite);
	if( len == -1) {
		if(errno == EAGAIN) {
			perror("Write to serial port would have blocked");
			return 0;
		}
		perror("Writing to serial port");
		return -1;
	}
//    DP("Done with cps_write()\n");
	return len;
}
 
