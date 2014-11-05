#ifndef USBCOMM_H_
#define USBCOMM_H_

/**
 * This file defines the low level communications interface with the robot
 *
 * \author Michael Schwerin
 */


#include "state.h"
#include "cps_thread.h"  // for TRUE FALSE definitions
#include <stdio.h>

#define USB_ESC 0x1B
// characters per transmit slice
#define TX_CPT 64

// size of buffer and number of bytes to read per time slice
#define USB_READ_BUF_SIZE 2048
#define USB_READ_SIZE 2048

#include "crossplatformserial.h"

enum tx_state_type {TXS_STARTED, TXS_DONE, TXS_WAIT};
enum tx_dest_type {TXD_BASE, TXD_OUTER};

typedef struct _sUSBComm {
	cpsPort port;

	int rxbytes;
	unsigned int DcuFlags;
	sState inState;
	unsigned char *inStateStr;
	int inpktlen;
	int escape;
	int rxbasecount;
	int rxoutercount;
	int bytes;
	int crc_errors;
	sState baseState, outerState;

	unsigned int lasttimeslice;		// Used for fakesnake
	unsigned int txcount;
	int txbytes;
	int base_pkt_num;
	int outer_pkt_num;
	sCmd txCmd;
	unsigned char *txCmdStr;
	int txpktlen;
	sCmd baseCmd, outerCmd;
	enum tx_state_type tx_state;
	enum tx_dest_type tx_nextdest;
} sUSBComm, *psUSBComm;


/**
 * copy the passed command packets to the inner state
 * \param state holds the state of the USB connection
 * \param inBase the command object with the base as destination
 * \param inOuter the command object with the outer board as destination
 * \return 0 on success
 */
int usbcomm_sendCmds(sUSBComm *state, sCmd* inBase, sCmd* inOuter);

/**
 * copy the received sState packets from the USB state to the passed sState objects
 * \param state holds the state of the USB connection
 * \param outBase the sState object with the base as source
 * \param outOuter the sState object with the outer board as source
 * \return 0 on success
 */
int usbcomm_getStates(sUSBComm *state, sState* outBase, sState* outOuter);

/**
 * This function initializes all the values in the command structure
 * \param inCmd a pointer to a sCmd object which will be initialized
 */
void init_sCmd(sCmd *inCmd);

/**
 * sets up all the initial values of the state.  Must be called before using
 * the sUSBComm object for any other calls
 * \param state the object to be initialized
 * \return 0 on success, -1 if a parameter is null
 */
int usbcomm_init(sUSBComm *state);

/**
 * Opens the named port and prepare it for use
 * \param state holds the state of the USB connection
 * \param portname the name of the COM/tty port to open
 * \return 0 on success, -1 if a parameter is null, -2 if port is not initially close, -3 if unable to open the port
 */
int usbcomm_open(sUSBComm *state, char *portname);

/**
 * Clean up and close the port associated with this sUSBComm object
 * \param state holds the state of the USB connection
 * \return 0 on success, -1 if a parameter is null, -2 if port is not initially open, -3 if couldn't close port
 */
int usbcomm_close(sUSBComm *state);

/**
 * Performs non-blocking IO on the port
 * \param state holds the state of the USB connection
 * \return 0 on success, -1 if a parameter is null
 */
int usbcomm_timeslice(sUSBComm *state);

void usbcomm_printState(FILE *outfile, sState *in);
void usbcomm_printMotorState(FILE *outfile, sMotorState *in);
int usbcomm_stateWordSwap(sState *in);
double double_wordswap(double in);
void usbcomm_printCmd(FILE *outfile, sCmd *in);
void usbcomm_printMotorCmd(FILE *outfile, sMotorCmd *in);
int usbcomm_cmdWordSwap(sCmd *in);
void usbcomm_statusPrint(sUSBComm *state);

#endif /*STATE_H_*/
