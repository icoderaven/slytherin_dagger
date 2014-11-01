#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../system/crc32.h"
#include "../system/state.h"
#include "../communication_interface/crossplatformserial.h"
#include "../communication_interface/usbcomm.h"
#include <assert.h>
#include "debug.h"
#include "../probe_control/CardioError.h"
#include "../probe_control/superCommon.h"
#include "../probe_control/config_defaults.h"

extern int TestBaseRelay;
extern int TestOuterRelay;
extern int TestVoltRelay;
extern int Buzzers_on;


#define VER_NUM 0x00020000
extern BOOL InnerSnakeLock;
extern BOOL OuterSnakeLock;
//#define VERBOSE


// Initializes a command structure to default values.
void init_sCmd(sCmd *inCmd) {
	int i;

	assert(inCmd != NULL);
	inCmd->padding = 0;
	inCmd->len = sizeof(g_cmd);
	inCmd->firmware_version = VER_NUM;
	inCmd->destination = SD_HOST;
	for(i = 0; i < 3; i++) {
		inCmd->motors[i].cmd_seq_number = 0;
		inCmd->motors[i].K_P = 0.5;			// takes 3200 tick error to saturate controller
		inCmd->motors[i].K_I = 0.0;
		inCmd->motors[i].K_D = 0.0;
		inCmd->motors[i].goal = 0;
		inCmd->motors[i].Ta = 50;			// .2 sec
		inCmd->motors[i].Tg = 125;			// .5 sec
		inCmd->motors[i].cur_limit = 155;	// 155 * 322.265 uA = 50 mA
		inCmd->motors[i].cur_inc = 1;		// 800 loop iterations to go to full current (3.2 seconds)
		inCmd->motors[i].cur_dec = 10;		// 160 loop iterations to turn fully off (.64 seconds)
		inCmd->motors[i].mtr_bits = MTR_BITS_TRACKING;      // default to set position tracking ON
		inCmd->motors[i].zeropos = 0;
	}
	inCmd->pkt_seq_number = 0;
}
static DOUBLE send_time = 0;


// Prints the status contained in a usbcomm state structure
void usbcomm_statusPrint(sUSBComm *state) {
	assert(state != NULL);
	printf("USB(hrx=%i btx=%i brx=%i diff=%i c=%i er=%i) COMM(brx=%i btx=%i bcrc=%i bovr=%i bfrm=%i | orx=%i otx=%i ocrc=%i oovr=%i ofrm=%i) m2Quad=0x%08X\n",
		state->rxbasecount+state->rxoutercount, state->baseState.usb_tx_count, state->baseState.usb_rx_count, state->rxbasecount+state->rxoutercount - state->baseState.usb_tx_count, state->rxbytes, state->crc_errors + state->baseState.usb_rx_error,
		state->baseState.comm_rx_count, state->baseState.comm_tx_count, state->baseState.comm_crc_error, state->baseState.comm_ovre_error, state->baseState.comm_frame_error,
		state->outerState.comm_rx_count, state->outerState.comm_tx_count, state->outerState.comm_crc_error, state->outerState.comm_ovre_error, state->outerState.comm_frame_error, state->baseState.motors[2].quadcount
					  	  );
}


// Initializes a usbcomm state structure
int usbcomm_init(sUSBComm *state) {
	assert(state != NULL);

	// general
	state->port.status = CPS_CLOSED;

	// rx related
	state->rxbytes = 0;
	state->inStateStr = (unsigned char*)&(state->inState);
	state->inpktlen = sizeof(sState);
	state->escape = FALSE;
	state->rxbasecount = 0;
	state->rxoutercount = 0;
	state->crc_errors = 0;

	// tx related
	state->txbytes = 0;
	state->base_pkt_num = 0;
	state->outer_pkt_num = 0;
	state->txCmdStr = (unsigned char*)&(state->txCmd);
	state->txpktlen = sizeof(sCmd);
	state->tx_state = TXS_WAIT;
	state->tx_nextdest = TXD_BASE;
	send_time = GetTickCount();

	return 0;
}


// Opens the port specified
int usbcomm_open(sUSBComm *state, char *portname) {
	assert(state != NULL);
	assert(portname != NULL);

	if(state->port.status != CPS_CLOSED) return -2;

	pCpsPort port = &(state->port);
  	port->status = CPS_CLOSED;

  	if( cps_OpenPort( port, B230400, portname ) == -1 ) {
        SetErrorBit(M_DCU,DCU_ERR_INIT_SERIAL);
    	fprintf(stderr, "could not open port %s\n", portname);
    	return -3;
	}
	return 0;
}


// Closes the specified port
int usbcomm_close(sUSBComm *state) {
	assert(state != NULL);
	if(state->port.status != CPS_OPENED) return -2;
	if( cps_ClosePort( &(state->port) ) == TRUE )
		return 0;
	return -3;
}

// Computes a 32 bit crc in the given data
unsigned int doChecksum(unsigned char *inStr, int len) {
	assert(inStr != NULL);
	unsigned int workCRC;
	int j;
	crc32_start(&workCRC);
	for(j = 0; j < len; j++) {
		crc32_update(inStr[j], &workCRC);
	}
	crc32_finish(&workCRC);
	return workCRC;
}


// Proccesses transmit and receive packets for the given state. This is where
// most of the work is done
int usbcomm_timeslice(sUSBComm *state) {
	assert(state != NULL);

	// Handle all the various error bits
	if (state->DcuFlags & ERROR_MASK) {
        // If othere's an error detected, disable that processor
        state->baseCmd.SafetyCmd &= ~SFT_C_ENABLE;
        state->outerCmd.SafetyCmd &= ~SFT_C_ENABLE;
    }
    else {
         // Otherwise enable if we're not testing that relay
		 if (TestBaseRelay) {
			state->baseCmd.SafetyCmd &= ~SFT_C_ENABLE;
		 }
		 else {
			 state->baseCmd.SafetyCmd |= SFT_C_ENABLE;
		 }

		 if (TestOuterRelay) {
			 state->outerCmd.SafetyCmd &= ~SFT_C_ENABLE;
		 }
		 else {
			 state->outerCmd.SafetyCmd |= SFT_C_ENABLE;
		 }

		 //Allow testing of the voltage monitors if we don't have any errors
		 if (TestVoltRelay) {
		 	state->baseCmd.SafetyCmd |= SFT_C_VOLT_TEST;
		 	state->outerCmd.SafetyCmd |= SFT_C_VOLT_TEST;
		 }
		 else {
		 	state->baseCmd.SafetyCmd &= ~SFT_C_VOLT_TEST;
		 	state->outerCmd.SafetyCmd &= ~SFT_C_VOLT_TEST;
		 }
    }

	// Turn on or off the buzzers as requested by the supervisor
	if (Buzzers_on) {
        state->baseCmd.SafetyCmd |= SFT_C_BUZZER;
        state->outerCmd.SafetyCmd |= SFT_C_BUZZER;
	}
	else {
        state->baseCmd.SafetyCmd &= ~SFT_C_BUZZER;
        state->outerCmd.SafetyCmd &= ~SFT_C_BUZZER;
	}

    if (state->DcuFlags & CLEAR_BIT) {
        state->baseCmd.SafetyCmd |= SFT_C_CLR_ERR;
        state->outerCmd.SafetyCmd |= SFT_C_CLR_ERR;
    }
    else {
        state->outerCmd.SafetyCmd &= ~SFT_C_CLR_ERR;
        state->baseCmd.SafetyCmd &= ~SFT_C_CLR_ERR;
    }
    if (state->DcuFlags & RESET_BIT) {
        state->baseCmd.SafetyCmd |= SFT_C_RESET;
    }
    else {
        state->baseCmd.SafetyCmd &= ~SFT_C_RESET;
    }

	if(state->port.status != CPS_OPENED) {
		// Not connected - try to connect
		SetErrorBit(M_DCU, DCU_ERR_READ_SERIAL);
		if (usbcomm_open(state, COM_PORT_NAME) < 0) {
			return -1;
		}
	}

	char cpBuf[USB_READ_BUF_SIZE];
	unsigned char *crcStr;
	unsigned int workCRC;
	int count, i, j;

	//////////////////////// READ //////////////////////////////
	count = cps_Read( &(state->port), cpBuf, USB_READ_SIZE, 1 );	// try reading for 1ms and limit to 128 bytes
	
    if(count > 0) {
        state->rxbytes += count;
    }
    for(i = 0; i < count; i++) {
		assert(state->inpktlen > 0); // shouldn't be zero until after processing cpBuf[i]
		assert((unsigned int)state->inpktlen <= sizeof(sState)); // if this fails we walked off the low side of the array
		if(cpBuf[i] == USB_ESC) {
	    	if(state->escape) {
				// double escape so put escape char into output buffer

				state->inStateStr[sizeof(sState)-state->inpktlen] = USB_ESC;
				state->inpktlen--;
				state->escape = false;
			}
			else {
				state->escape = true;
			}
		}
		else {
			if( state->escape ) { // if previous char was escape and this isn't then this is start of a packet
				state->escape = false;
				state->inStateStr = (unsigned char *)&(state->inState);
				state->inpktlen = sizeof(sState);
			}
			// just a normal char to store (if previous was escape then this is the first)
			state->inStateStr[sizeof(sState)-state->inpktlen] = cpBuf[i];
			state->inpktlen--;
		}
		//printf("inpktlen = %d\n", state->inpktlen);
		if(state->inpktlen == 0) {
                           //printf("HAVE A PACKET!\n");
			// we have a complete packet so verify the checksum
			workCRC = doChecksum((unsigned char *)&(state->inState), sizeof(sState)-sizeof(int) );
			if(workCRC != state->inState.crc32) {
				// checksum test failure
				state->crc_errors++;
			} else {
				if( (state->inState.len == sizeof(sCmd)) &&
				   ((state->rxbasecount+state->rxoutercount) & 0x1E) == 0 )
				{ // this is a feedback test so print it out
					sCmd *work = (sCmd*)&(state->inState);
					usbcomm_cmdWordSwap(work);
					usbcomm_printCmd(stdout, work);
				}
				// checksum matched so put the packet where it goes
				if( (state->inState.source == SD_BASE) ) {
					state->rxbasecount++;
					memcpy(&(state->baseState), &(state->inState), sizeof(sState));
				}
	  			if( (state->inState.source == SD_OUTER) ) {
	  				state->rxoutercount++;
					memcpy(&(state->outerState), &(state->inState), sizeof(sState));
	  			}

				#ifdef VERBOSE
				 if( ((state->rxbasecount+state->rxoutercount) & 0x1F) == 0 )
					printf("USB(hrx=%i btx=%i brx=%i diff=%i c=%i er=%i) COMM(brx=%i btx=%i bcrc=%i bovr=%i bfrm=%i | orx=%i otx=%i ocrc=%i oovr=%i ofrm=%i) Quad=0x%08X\n",
						state->rxbasecount+state->rxoutercount, state->baseState.usb_tx_count, state->baseState.usb_rx_count, state->rxbasecount+state->rxoutercount - state->baseState.usb_tx_count, state->rxbytes, state->crc_errors + state->baseState.usb_rx_error,
						state->baseState.comm_rx_count, state->baseState.comm_tx_count, state->baseState.comm_crc_error, state->baseState.comm_ovre_error, state->baseState.comm_frame_error,
						state->outerState.comm_rx_count, state->outerState.comm_tx_count, state->outerState.comm_crc_error, state->outerState.comm_ovre_error, state->outerState.comm_frame_error, state->baseState.motors[0].quadcount
						);
				#endif
			}
			// set up next read
        	state->inStateStr = (unsigned char *) &(state->inState);
        	state->inpktlen = sizeof(sState);
		}  /* if(pktlen == 0) */
	} /* for(i = 0; i < count; i++) */

    ///////////////////////////////////////// WRITE ////////////////////////////////////
    if (GetTickCount() > send_time)
    {
    if(state->tx_state != TXS_WAIT) {
    	unsigned char outBuf[TX_CPT];
    	int i = 0;

    	// do we need to start sending a new packet?
    	if(state->tx_state == TXS_DONE) {
			state->tx_state = TXS_STARTED;
			outBuf[i++] = USB_ESC;			// the initial solo escape to signal start of packet
			if(state->tx_nextdest == TXD_BASE) {
				state->base_pkt_num++;
				state->baseCmd.pkt_seq_number = state->base_pkt_num;
				memcpy( &(state->txCmd), &(state->baseCmd), sizeof(sCmd));
				state->tx_nextdest = TXD_OUTER;
			} else  {
				state->outer_pkt_num++;
				state->outerCmd.pkt_seq_number = state->outer_pkt_num;
				memcpy( &(state->txCmd), &(state->outerCmd), sizeof(sCmd));
				state->tx_nextdest = TXD_BASE;
			}
			// calculate the CRC
			crcStr = (unsigned char *) &(state->txCmd);
			crc32_start(&workCRC);
			for(j = 0; (unsigned int)j < sizeof(sCmd) - sizeof(int); j++) {
				crc32_update(crcStr[j], &workCRC);
			}
			crc32_finish(&workCRC);
			state->txCmd.crc32 = workCRC;
			state->txCmdStr = (unsigned char*) &(state->txCmd);
			state->txpktlen = sizeof(sCmd);
		}
    	while( (i < TX_CPT) &&   // two end conditions for filling the buffer, either buffer full
			   (state->txpktlen > 0) )	// or all the data is sent
		{
			if(*state->txCmdStr == USB_ESC) {  // if the next char to send is and escape we need to send a double escape sequence
				if(i + 1 < TX_CPT) { // make sure there is enough room in this packet to send the double escape sequence
					outBuf[i++] = USB_ESC;
					outBuf[i++] = USB_ESC;
					state->txpktlen--; // one less byte to send in the state structure
					state->txCmdStr++; // move pointer to next char in state structure to send
				}
				else 		// if not enough space then we give up here and just send the double escape
					break;	// as start of next packet
			}
			else { // just a normal (non escape) char so just add it to the buffer and update the remaining count
				outBuf[i++] = *state->txCmdStr++;
				state->txpktlen--;
			}
		}
		if(state->txpktlen == 0) {	// if we hit the end of the packet to be transmitted then update state
			state->tx_state = TXS_DONE;
			send_time = GetTickCount() + 20;
		}
		// send the i bytes in the packet
		assert(i>0);
		assert(i <= TX_CPT);
		if (cps_Write( &(state->port), (char *)outBuf, i) < 0) {
			// Close the port if the write failed
			usbcomm_close(state);
		}
    }
    }
    return 0;
}

int usbcomm_sendCmds(sUSBComm *state, sCmd* inBase, sCmd* inOuter) {
	assert(state != NULL);
	if(inBase != NULL) {
		memcpy( &(state->baseCmd), inBase, sizeof(sCmd) );
	}
	if(inOuter != NULL) {
		memcpy( &(state->outerCmd), inOuter, sizeof(sCmd) );
	}
	if(state->tx_state == TXS_WAIT) {
		state->tx_state = TXS_DONE;		// move out of the wait state once we have stuff to transmit
	}
	return 0;
}

void usbcomm_printMotorState(FILE *outfile, sMotorState *in) {
	assert(in != NULL);
	fprintf(outfile, "integral=%e error=%e lasterror=%e quadcount=0x%08X goal=0x%08X ", (in->integral), (in->error), (in->lasterror), in->quadcount, in->goal);
	fprintf(outfile, "seq=%i pwm=%i current=%i pwm_limit=%i\n", in->cmd_seq_last, in->pwm, in->current, in->pwm_limit);
	fprintf(outfile, "   traj_goal=%e traj_vel=%e traj_velmax=%e traj_accel=%e traj_ticks=%i traj_state=0x%X",
	 (in->traj_goal), (in->traj_vel), (in->traj_velmax), (in->traj_accel), in->traj_ticks, in->traj_state);
}

/* print Communication, and Safety fields of State packets
 *
 */
void usbcomm_printState(FILE *outfile, sState *in) {
	assert(in != NULL);

	// Source, Timestamp, comm rx count, comm rx lastseq, comm tx count, comm crc err,  comm overrun, comm frame err, usb rx count, usb rx error, usb rx lastseq, usb tx_ count \n");
	fprintf(outfile, "%i, ",  in->timestamp);

	fprintf(outfile, "%i, %i, %i, %i, %i, %i",
 	      in->comm_rx_count, in->comm_rx_lastseq, in->comm_tx_count, in->comm_crc_error, in->comm_ovre_error, in->comm_frame_error );
	if(in->source == SD_BASE)
	{
		fprintf(outfile, ",%i, %i, %i, %i",
	       in->usb_rx_count, in->usb_rx_error, in->usb_rx_lastseq, in->usb_tx_count);
	}
	fprintf(outfile, "\n");
	fflush(outfile);
}

void usbcomm_printMotorCmd(FILE *outfile, sMotorCmd *in) {
	assert(in != NULL);
	fprintf(outfile, ", %i, %e, %e, %e, %08d, %i, %i, %i, %i, %i, %i",
	       in->cmd_seq_number, in->K_P, in->K_I, in->K_D, in->goal, in->Ta, in->Tg, in->cur_limit, in->cur_inc, in->cur_dec, in->mtr_bits);
}


void usbcomm_printCmd(FILE *outfile, sCmd *in) {
	assert(in != NULL);
	int i;
	fprintf(outfile, " %i, %i, %x ", GetTickCount(), in->pkt_seq_number, in->SafetyCmd);
	for(i = 0; i < 3; i++) {
		usbcomm_printMotorCmd(outfile, &(in->motors[i]));
	}
	fprintf(outfile, "\n");
	fflush(outfile);
}

int usbcomm_getStates(sUSBComm *state, sState* outBase, sState* outOuter) {
	assert(state != NULL);
	if(outBase != NULL)
		memcpy(outBase, &(state->baseState), sizeof(sState) );
	if(outOuter != NULL)
		memcpy(outOuter, &(state->outerState), sizeof(sState) );
	return 0;
}

double double_wordswap(double in) {
	unsigned int *workStrI, *workStrO;
	double out;
	workStrI = (unsigned int *)&in;
	workStrO = (unsigned int *)&out;
	workStrO[1] = workStrI[0];
	workStrO[0] = workStrI[1];
	return out;
}

int usbcomm_stateWordSwap(sState *in) {
    int i;
	if(in == NULL) return -1;
	for(i = 0; i < 3; i++) {
		in->motors[i].integral = double_wordswap(in->motors[i].integral);
		in->motors[i].error = double_wordswap(in->motors[i].error);
		in->motors[i].lasterror = double_wordswap(in->motors[i].lasterror);
		in->motors[i].traj_goal = double_wordswap(in->motors[i].traj_goal);
		in->motors[i].traj_vel = double_wordswap(in->motors[i].traj_vel);
		in->motors[i].traj_velmax = double_wordswap(in->motors[i].traj_velmax);
		in->motors[i].traj_accel = double_wordswap(in->motors[i].traj_accel);
	}
	return 0;
}

int usbcomm_cmdWordSwap(sCmd *in) {
    int i;
	if(in == NULL) return -1;
	for(i = 0; i < 3; i++) {
		in->motors[i].K_P = (float)double_wordswap(in->motors[i].K_P);
		in->motors[i].K_I = (float)double_wordswap(in->motors[i].K_I);
		in->motors[i].K_D = (float)double_wordswap(in->motors[i].K_D);
	}
	return 0;
}
