#include <string.h> // For memset
#include "usbcomm.h"
#include "guided_motions.h"
#include "CardioError.h"
#include "superCommon.h"
#include "supervisor.h"
#include "ErrMessages.h"
#include "log.h"

// Indicates all relays were tested successfully
#define RELAY_TEST_SUCCESS 0
// Indicates a relay failed in the relay test
#define RELAY_TEST_FAIL -1
// Indicates the relay test is still in progress and the function needs to be
// called again
#define RELAY_TEST_CONTINUE 1

// Timeout and number of program cycles (both must be exceeded) before we assume
// a relay has failed
#define RELAY_TIMEOUT 1000
#define RELAY_CYCLES 5


// Constants use to keep track of relay test state
#define RELAY_TEST_IDLE  0
#define RELAY_TEST_INIT_DCU   1
#define RELAY_TEST_DCU   2
#define RELAY_TEST_INIT_BASE  3
#define RELAY_TEST_BASE  4
#define RELAY_TEST_INIT_OUTER 5
#define RELAY_TEST_OUTER 6
#define RELAY_TEST_INIT_VMON  7
#define RELAY_TEST_VMON  8
#define RELAY_TEST_DONE  9
#define RELAY_CLOSE 1
#define RELAY_OPEN 2
#define RELAY_NO_WAIT 0


// Number of ms since system boot beyond which will generate an warning.
// Set to zero to disable
//#define MAX_MS_WARN 86400000 // 24 hours
#define MAX_MS_WARN 0 // 0 hours
// Number of ms since system boot beyond which will generate a non-recoverable
// error. Set to zero to disable
//#define MAX_MS_ERROR 172800000 // 48 hours
#define MAX_MS_ERROR 0 // 0 hours

// What firmware version this version of the DCU software works with
//#define COMPATIBLE_FW_VERSION 0x00040003
#define COMPATIBLE_FW_VERSION 262149


//suport routines for DCU software supervisor

// Variables visable to all subroutines in this file
unsigned int ErrorState;        //variable to record state for error handling state machine
DWORD BPErrorFlags, OPErrorFlags;
ProcErr ErrP;
ProcErr ErrMask;	// Masks off errors we've seen before, so we only print once
DWORD PrevBadPacketCount;
DWORD PrevBaseSequenceNumber,PrevOuterSequenceNumber;
DWORD PresentJoyStickPacketNumber,PrevJoystickPacketNumber;
int Qindex;
float CurrentJoyX,CurrentJoyY;
CTimer vTimer[N_TIMERS];
bool RecoverFlag;
sUSBComm *      Pusb_state;
int TestBaseRelay = 0;
int TestOuterRelay = 0;
int TestVoltRelay = 0;
int RelayTestDone = 0;
int Buzzers_on = 0;

// Keeps us from warning about max uptime again and again and again...
static int max_ms_warn_issued = 0;

// Keeps track of if we've cleared all errors during recovery
static int cleared_errors = 0;

/**********************************************************
   SetErrorBit - set the errror bit in the appropriate word
************************************************************/

int SetErrorBit1(unsigned short source, unsigned int mask){
    printf("SetErrorBit(source=%u, mask=0x%x)\n", source, mask);
    int status = OKAY;
    switch(source){
        case M_DCU:
             ErrP.DcuDword |= mask;
             break;
        case M_BP:
             ErrP.BaseDword |= mask;
             break;
        case M_OP:
             ErrP.OuterDword |= mask;
             break;
		case M_MOT_B1:
		case M_MOT_B2:
		case M_MOT_B3:
		case M_MOT_O1:
		case M_MOT_O2:
		case M_MOT_O3:
			ErrP.MotorDword[source - M_MOT_B1] |= mask;
        default:
            break;
    }
    return status;
}


/************************************************************
	CloseSafetyCircuit_Errors - Closes safety circuit and sets error bits if
	there's an error.
************************************************************/
static void CloseSafetyCircuit_Errors(void) {
	if (CloseSafetyCircuit() == SERR) {
		ErrP.DcuDword |= DCU_ERR_KEITHLEY;
		SetErrorBit(M_DCU, DCU_ERR_KEITHLEY);
		Pusb_state->DcuFlags |= DCU_ERROR_BIT;
		QueueMessage(M_DCU,&ErrP);
		ResetDCUIOBoard();
	}
}


/************************************************************
	OpenSafetyCircuit_Errors - Opens safety circuit and sets error bits if
	there's an error.
************************************************************/
static void OpenSafetyCircuit_Errors(void) {
	if (OpenSafetyCircuit() == SERR) {
		ErrP.DcuDword |= DCU_ERR_KEITHLEY;
		SetErrorBit(M_DCU, DCU_ERR_KEITHLEY);
		Pusb_state->DcuFlags |= DCU_ERROR_BIT;
		QueueMessage(M_DCU,&ErrP);
		ResetDCUIOBoard();
	}
}

/************************************************************
	CheckSafetyCircuit_Errors - Checks the safe circuit and sets error bits if
	there's an error.
************************************************************/
static void CheckSafetyCircuit_Errors(long *state) {
	if (CheckSafetyCircuit(state) == SERR) {
		ErrP.DcuDword |= DCU_ERR_KEITHLEY;
		SetErrorBit(M_DCU, DCU_ERR_KEITHLEY);
		Pusb_state->DcuFlags |= DCU_ERROR_BIT;
		QueueMessage(M_DCU,&ErrP);
		ResetDCUIOBoard();
	}
}

/************************************************************
	CheckEstopButton_Errors - Checks the safe circuit and sets error bits if
	there's an error.
************************************************************/
static void CheckEstopButton_Errors(long *state) {
	if (CheckEstopButton(state) == SERR) {
		ErrP.DcuDword |= DCU_ERR_KEITHLEY;
		SetErrorBit(M_DCU, DCU_ERR_KEITHLEY);
		Pusb_state->DcuFlags |= DCU_ERROR_BIT;
		QueueMessage(M_DCU,&ErrP);
		ResetDCUIOBoard();
	}
}

/***************************************************************
RunSupervisor - main supervisor routine

***************************************************************/

int RunSupervisor(unsigned int Action){
    int TimerMask,status,k;
    DWORD Diff;
    long SafetyState = 0;
	int seq_diff;
	unsigned int ticks;
	JOYINFO dummy_joy_info;

    status = OKAY;
    switch(Action){
      case INIT_SUPERVISOR:
           //Init variables for supervisor
		   // Clear error mask
		    memset(&ErrMask, 0, sizeof(ErrMask));
            ErrorState = SAFE_STATE;
            Pusb_state->DcuFlags = DCU_ERROR_BIT;
            Qindex = 0;
            BPErrorFlags = 0;
            OPErrorFlags = 0;
            PrevBadPacketCount = 0;
            PresentJoyStickPacketNumber = 0;
            PrevJoystickPacketNumber = 0;
            InitTimers();
            //clear message queue
            for(k = 0; k < NUM_MESSAGES; k++){
                  CMessQ[k].Valid = FALSE;
            }
            OpenSafetyCircuit_Errors();
            RecoverFlag = FALSE;
           break;
      case RUN_SUPERVISOR:
	       // Check for system booted for too long
		   ticks = GetTickCount();
		   if (ticks > MAX_MS_ERROR && MAX_MS_ERROR != 0) {	// Check error first
		   		SetErrorBit(M_DCU, DCU_ERR_UPTIME);
		   }
		   else if (ticks > MAX_MS_WARN && MAX_MS_WARN != 0 && (!max_ms_warn_issued)) { // If not an error, check for warning
		   		max_ms_warn_issued = 1;
		   		SetErrorBit(M_DCU, DCU_WRN_UPTIME);
		   }

		   // Check for invalid firwmare version (only if we've received a
		   // packet from that processor)
		   if ((Pusb_state->baseState.firmware_version != COMPATIBLE_FW_VERSION
		           && Pusb_state->rxbasecount > 0)
		           || (Pusb_state->outerState.firmware_version !=
				   COMPATIBLE_FW_VERSION && Pusb_state->rxoutercount > 0)) {
		       SetErrorBit(M_DCU, DCU_ERR_INCOMPAT_FW);
		   }


           // Check that the joystick is still there
		   if (joyGetPos(JOYSTICKID1, &dummy_joy_info) != JOYERR_NOERROR) {
		       SetErrorBit(M_DCU, DCU_ERR_NO_JOY);
		   }

          TimerMask = CheckTimers();
          //perform checks
          if(TimerMask & CHECK_PACKET_ERROR_RATE){
               Diff = Pusb_state->crc_errors - PrevBadPacketCount;
               PrevBadPacketCount = Pusb_state->crc_errors ;
               if(Diff > ACCEPTABLE_ERROR_RATE){
                    SetErrorBit(M_DCU,DCU_WRN_BAD_PACKETS);
               }
          }
          if(TimerMask & CHECK_PACKET_TIMEOUT){
		       // Check base processor
               Diff = Pusb_state->rxbasecount - PrevBaseSequenceNumber;
               PrevBaseSequenceNumber = Pusb_state->rxbasecount;
               if(Diff <= 0){
                    SetErrorBit(M_DCU,DCU_ERR_TIMEOUT_FDR);
               }

		       // Check outer processor
               Diff = Pusb_state->rxoutercount - PrevOuterSequenceNumber;
               PrevOuterSequenceNumber = Pusb_state->rxoutercount;
               if(Diff <= 0){
                    SetErrorBit(M_DCU,DCU_ERR_TIMEOUT_FDR);
               }
          }
          if(TimerMask & CHECK_JOYSTICK_TIMEOUT){
               Diff = PresentJoyStickPacketNumber - PrevJoystickPacketNumber;
               PrevJoystickPacketNumber =  PresentJoyStickPacketNumber;
               if(Diff <= 0){
                    //SetErrorBit(M_DCU,DCU_ERR_TIMEOUT_JOY);
               }
          }
		  // Check sequence numbers
		  for (k = 0; k < 3; k++) {
		      seq_diff = Pusb_state->baseCmd.motors[k].cmd_seq_number -
			      Pusb_state->baseState.motors[k].cmd_seq_last;
			  if (seq_diff < 0 || seq_diff > MAX_SEQ_DIFF) {
                  ErrP.DcuDword |= DCU_ERR_ILLEGAL_SEQ;
			      SetErrorBit(M_DCU, DCU_ERR_ILLEGAL_SEQ);
              }

		      seq_diff = Pusb_state->outerCmd.motors[k].cmd_seq_number -
			      Pusb_state->outerState.motors[k].cmd_seq_last;
			  if (seq_diff < 0 || seq_diff > MAX_SEQ_DIFF) {
                  ErrP.DcuDword |= DCU_ERR_ILLEGAL_SEQ;
			      SetErrorBit(M_DCU, DCU_ERR_ILLEGAL_SEQ);
              }
          }


          if (ErrorState == RUN_STATE) {
              CheckSafetyCircuit_Errors(&SafetyState);
			  if(!SafetyState){
			      ErrP.DcuDword |= DCU_ERR_E_STOP;
				  SetErrorBit(M_DCU, DCU_ERR_E_STOP);
              }

              CheckEstopButton_Errors(&SafetyState);
			  if(!SafetyState) {
			  	ErrP.DcuDword |= DCU_ERR_E_STOP_PRESSED;
				SetErrorBit(M_DCU, DCU_ERR_E_STOP_PRESSED);
			  }
          }
          status = ErrorStateMachine();
          OutPutMessages();             //print out messages
          break;
     default:
          break;
     }
     return status;
}


/******************************************************
   InitTimers() -routine to initialize timers
   timer 0 - check packet error rate
   timer 1 - check for packet from feeder
   timer 2 - check for joystick data
*********************************************************/

int InitTimers(){
    int status = OKAY;
    int k;
    long Time;

    Time = GetTickCount();
    for(k = 0; k < N_TIMERS; k++){
          vTimer[k].Valid = 1;
          vTimer[k].FinalValue = Time + INITIAL_INTERVAL;
    }
    return status;
}


/*********************************************************
Routine to check timers:
        Check packet error rate every second (timer 0)
        Check for packet from feeder within 150 msec (timer 1)
        Check for data from joystick (timer 2)
********************************************************/
int CheckTimers(){
    int TimerMask=0,mask=1;
    int k;

    for(k = 0; k < N_TIMERS; k++){
          if(vTimer[k].Valid){
              if(GetTickCount() > vTimer[k].FinalValue){
                  TimerMask |= mask;
                  vTimer[k].FinalValue = GetTickCount() + TimeoutVal[k];
                  vTimer[k].Valid = TRUE;
              }
          }
          mask = mask << 1;
    }
    return TimerMask;
}


int PrintErrors() {
	bool ErrorFlag = FALSE;
	int k, motor_index;

	//copy in error flags
	ErrP.BaseDword = Pusb_state->baseState.SafetyError;
	ErrP.OuterDword = Pusb_state->outerState.SafetyError;
	if(ErrP.DcuDword & DCU_ERROR_MASK){
	  	 QueueMessage(M_DCU,&ErrP);
	  	 // Zero out all recoverable errors/warnings
	  	 ErrP.DcuDword &= DCU_NO_RECOVER;
	  	 ErrorFlag = TRUE;
	  	 Pusb_state->DcuFlags |= DCU_ERROR_BIT;
		 printf("Found DCU error\n");
	}
	if(ErrP.BaseDword & SFT_SE_ERROR_MASK){
	  	QueueMessage(M_BP,&ErrP);
	  	ErrP.BaseDword = 0;
	  	ErrorFlag = TRUE;
	  	Pusb_state->DcuFlags |= BP_ERROR_BIT;
		 printf("Found BP error\n");
	}
	if(ErrP.OuterDword & SFT_SE_ERROR_MASK){
	  	QueueMessage(M_OP,&ErrP);
	  	ErrP.OuterDword = 0;
	  	ErrorFlag = TRUE;
	  	Pusb_state->DcuFlags |= OP_ERROR_BIT;
		 printf("Found OP error\n");
	}
	// Check the motors for errors
	for(k = 0; k < 3; k++){
	  	if(Pusb_state->baseState.motors[k].SafetyError & SFT_SE_MOTOR_MASK){
	  		  motor_index = M_MOT_B1 + k;
	  		  SetErrorBit(motor_index, Pusb_state->baseState.motors[k].SafetyError);
	  		  QueueMessage(motor_index,&ErrP);
			  ErrP.MotorDword[motor_index] = 0;
	  		  ErrorFlag = TRUE;
	  		  Pusb_state->DcuFlags |= BP_ERROR_BIT;
			 printf("Found BP motor error\n");
	  	}
	  	if(Pusb_state->outerState.motors[k].SafetyError & SFT_SE_MOTOR_MASK){
	  		  motor_index = M_MOT_B1 + k + 3;
	  		  SetErrorBit(motor_index, Pusb_state->outerState.motors[k].SafetyError);
	  		  QueueMessage(motor_index,&ErrP);
			  ErrP.MotorDword[motor_index] = 0;
	  		  ErrorFlag = TRUE;
	  		  Pusb_state->DcuFlags |= OP_ERROR_BIT;
			  printf("Found OP motor error\n");
	  	}
	}
	//check for warnings. Just queue message
	if(ErrP.DcuDword & DCU_WRN_MASK){
	  	QueueMessage(M_DCU,&ErrP);
	}
	if(ErrP.BaseDword & SFT_SE_WARN_MASK){
	  	QueueMessage(M_BP,&ErrP);
	}
	if(ErrP.OuterDword & SFT_SE_WARN_MASK){
	  	QueueMessage(M_OP,&ErrP);
	}

	return ErrorFlag;
}


//***************************************************************************
// ErrorStateMachine
//
// Manages the error state machine that handles warnings and errors
//***************************************************************************
int ErrorStateMachine(){
    bool  ErrorFlag = FALSE;
    int status=OKAY;
    long int safety_circuit_stat = 0;

    switch(ErrorState){

         case RUN_STATE:
              // Check warning and error bits.
              // queue printf and log for warnings, sound buzzer
              // propogate errors
              // set new state
              //Check for errors for all processors
              ErrorState = RUN_STATE;

			  // Turn off the buzzers when we're in run state
			  Buzzers_on = 0;

			  if (PrintErrors()) ErrorFlag = true;
			  else ErrorFlag = false;

              //Update the state
              if(ErrorFlag){
                  ErrorState = ERROR_STATE;
                  ErrorFlag = FALSE;
                  OpenSafetyCircuit_Errors();

                  pause();    //pause the system
				  set_mode(ABANDON); // abandon our most recent movement
              }
              break;
         case ERROR_STATE:
              // printf("ERROR_STATE\n");
              ErrorState = ERROR_STATE;

			  // If we're in error state turn on both buzzers
			  Buzzers_on = 1;

			  if (PrintErrors()) ErrorFlag = true;
			  else ErrorFlag = false;

              // printf("dcuFlags=%x, base safety err=%x, mot1 err=%x, mot2 err=%x mot3 err=%x, outer safety err=%x, mot4 err=%x, mot5 err=%x mot6 err=%x\n", Pusb_state->DcuFlags, Pusb_state->baseState.SafetyError, Pusb_state->baseState.motors[0].SafetyError, Pusb_state->baseState.motors[1].SafetyError, Pusb_state->baseState.motors[2].SafetyError, Pusb_state->outerState.SafetyError, Pusb_state->outerState.motors[0].SafetyError, Pusb_state->outerState.motors[1].SafetyError, Pusb_state->outerState.motors[2].SafetyError);
              if((Pusb_state->DcuFlags & ERROR_MASK)&& (Pusb_state->baseState.SafetyError & SFT_SE_SAFE_STATE)
                   && (Pusb_state->outerState.SafetyError & SFT_SE_SAFE_STATE)){

					// Set us to the appropriate safe state depending on if we
					// can recover or not
                    if (ErrP.DcuDword & DCU_NO_RECOVER) {
						// all processors acknowledge the error, but we cannot recover
						ErrorState = SAFE_STATE_NO_RECOVER;
					}
					else {
						ErrorState = SAFE_STATE;   //all processors acknowledge error
					}
              }
              break;
         case SAFE_STATE:
              ErrorState = SAFE_STATE;
			  if (PrintErrors()) ErrorFlag = true;
			  else ErrorFlag = false;
              // operator wnats to return to run mode
              if(RecoverFlag){
                    RecoverFlag = FALSE;
                    // printf("Saw recover flag\n");
                    Pusb_state->DcuFlags = CLEAR_BIT;       //clear the processors
                    ErrorState = RECOVERY_STATE;
              }
              break;
		 // In safe state, but cannot recover. User will be unable to get out
		 // of this state
		 case SAFE_STATE_NO_RECOVER:
		 	  //printf("N");
			  ErrorState = SAFE_STATE_NO_RECOVER;
			  break;

         case RECOVERY_STATE:
              //printf("RECOVERY_STATE\n");
              ErrorState = RECOVERY_STATE;
			  // Zero out any errors before we check for more
			  // If errors still exist, PrintErrors() will put them back into
			  // DcuFlags
			  Pusb_state->DcuFlags = CLEAR_BIT;       //clear the processors
			  if (PrintErrors()) ErrorFlag = true;
			  else ErrorFlag = false;
              //printf("dcuFlags=%x, base safety err=%x, mot1 err=%x, mot2 err=%x mot3 err=%x, outer safety err=%x, mot4 err=%x, mot5 err=%x mot6 err=%x, cleared_errors=%d\n", Pusb_state->DcuFlags, Pusb_state->baseState.SafetyError, Pusb_state->baseState.motors[0].SafetyError, Pusb_state->baseState.motors[1].SafetyError, Pusb_state->baseState.motors[2].SafetyError, Pusb_state->outerState.SafetyError, Pusb_state->outerState.motors[0].SafetyError, Pusb_state->outerState.motors[1].SafetyError, Pusb_state->outerState.motors[2].SafetyError, cleared_errors);
              //printf("base safety err=%x, outer safety err=%x\n", Pusb_state->baseState.SafetyError, Pusb_state->outerState.SafetyError);
              if((!(Pusb_state->DcuFlags & ERROR_MASK)) && (!(Pusb_state->baseState.SafetyError & SFT_SE_ERROR_MASK))
                && (!(Pusb_state->outerState.SafetyError & SFT_SE_ERROR_MASK))) {
                     printf("Trying to reset safety circuit.\n");
					 // Turn off the clear bit
					 Pusb_state->DcuFlags &= ~CLEAR_BIT;
					 // Clear our error mask
					 memset(&ErrMask, 0, sizeof(ErrMask));

					// Turn on the DCU relay and reset the relay test if this
					// is our first turn-on
					if (!cleared_errors) {
						CloseSafetyCircuit_Errors();
						if (! RelayTestDone) {
							RelayTest(1);
						}
					}
					// remember that we successfully cleared all the errors
					cleared_errors = 1;

					// Run the relay test and wait until it finishes
					if (! RelayTestDone) {
						switch (RelayTest(0)) {
							case RELAY_TEST_CONTINUE:
								// Keep waiting
								return status;
								break;
							case RELAY_TEST_SUCCESS:
								// Success, we can enable the system
								RelayTestDone = 1;
								break;
							case RELAY_TEST_FAIL:
								// Failed, show the error
								RelayTestDone = 0;
								SetErrorBit(M_DCU, DCU_ERR_RELAY_FAIL);
								QueueMessage(M_DCU,&ErrP);
							    Pusb_state->DcuFlags |= DCU_ERROR_BIT;
								// go back to error state
								ErrorState = ERROR_STATE;
								ErrorFlag = FALSE;
								// Turn off the clear bit
								Pusb_state->DcuFlags &= ~CLEAR_BIT;
								OpenSafetyCircuit_Errors();

								pause();    //pause the system
								set_mode(ABANDON); // abandon our most recent movement
								cleared_errors = 0;
								return status;
								break;
							default:
								break;
						}
					}



					 // Wait until all relays are on before resetting
					 if (Pusb_state->baseState.SafetyStat & SFT_S_TEST_STAT) {
						 Pusb_state->DcuFlags |= RESET_BIT;
						 CheckSafetyCircuit_Errors(&safety_circuit_stat);

						 printf("2. ErrP.DcuDword = %x\n", ErrP.DcuDword);
						 // Do not reset safety circuit if we found any DCU
						 // errors while enabling the relays
						 if (safety_circuit_stat &&
						 		!(Pusb_state->DcuFlags & ERROR_MASK)) {

							printf("cleared_errors=0 in if\n");
							cleared_errors = 0;
							Pusb_state->DcuFlags = 0;
							ErrorState = RUN_STATE;
							ErrP.InfoDword |= RET_TO_RUN;
							QueueMessage(M_INFO, &ErrP);
							//guided_motions_reinit();
							pause();      //make sure we are in pause state
						 }
					}
              }
			  else if (cleared_errors) {
			  	// If we already cleared all the errors, we have a new one, so
				// go back to error state
			    ErrorState = ERROR_STATE;
			    ErrorFlag = FALSE;
				// Turn off the clear bit
			    Pusb_state->DcuFlags &= ~CLEAR_BIT;
			    OpenSafetyCircuit_Errors();

			    pause();    //pause the system
			    set_mode(ABANDON); // abandon our most recent movement
				printf("cleared_errors=0 in else\n");
				cleared_errors = 0;

			  }
              break;
         default:
                 break;
    }
    return status;
}

/****************************************************************
   ErrMsgIndex - find the index for the Error Message based on
   error bit set.
****************************************************************/

int ErrMsgIndex(DWORD ErrorWord){
    int k,size;

    size = 8 * sizeof(ErrorWord);    //number of bits in word
    k = 0;
    while(((ErrorWord & 0x1) == 0) && (k < size)){
         ErrorWord = ErrorWord >> 1;
         k++;
    }
    return k;
}

/****************************************************************
   QueueMessage

   Put an error message on the queue. It will be logged and displayed
   at end of loop.
******************************************************************/

int QueueMessage(unsigned short source, ProcErr *pErr){
    int status = OKAY;
    DWORD ErrorWord,mask=1;
    int k,Type,motor;
    char **MessPtr;

    switch(source){
    case M_DCU:
         MessPtr = DCU_Err_Messages;
         Type = LOG_ERROR;
         ErrorWord = pErr->DcuDword & ~ErrMask.DcuDword;
		 ErrMask.DcuDword |= pErr->DcuDword;
         pErr->DcuDword = 0;
         break;
    case M_BP:
         MessPtr = BP_Err_Messages;
         Type = LOG_ERROR;
         ErrorWord = pErr->BaseDword & ~ErrMask.BaseDword;
		 ErrMask.BaseDword |= pErr->BaseDword;
         pErr->BaseDword = 0;
         break;
    case M_OP:
         MessPtr = OP_Err_Messages;
         Type = LOG_ERROR;
         ErrorWord = pErr->OuterDword & ~ErrMask.OuterDword;
		 ErrMask.OuterDword |= pErr->OuterDword;
         pErr->OuterDword = 0;
         break;
    case M_INFO:
         MessPtr = DCU_Err_Messages;
         Type = LOG_INFO;
         ErrorWord = pErr->DcuDword & ~ErrMask.DcuDword;
		 ErrMask.DcuDword |= pErr->DcuDword;
         pErr->DcuDword = 0;
         break;
    case M_MOT_B1:
    case M_MOT_B2:
    case M_MOT_B3:
    case M_MOT_O1:
    case M_MOT_O2:
    case M_MOT_O3:
         MessPtr = MOT_Err_Messages;
         Type = LOG_ERROR;
         ErrorWord = pErr->MotorDword[source - M_MOT_B1] & ~ErrMask.MotorDword[source - M_MOT_B1];
		 ErrMask.MotorDword[source - M_MOT_B1] |= pErr->MotorDword[source - M_MOT_B1];
         pErr->MotorDword[source - M_MOT_B1] = 0;
		 break;
    default:
         status = SERR;
         return status;
    }
     // get all errors

     while(ErrorWord != 0){
	 	 printf("ErrorWord = 0x%x\n", (unsigned int)ErrorWord);
         if(CMessQ[Qindex].Valid){
              status = SERR;      //overflow of queue.Write over anyway
         }
         k = ErrMsgIndex(ErrorWord);
         ErrorWord = ErrorWord ^ (mask << k);  // zero out bit
         mask = 1;
         if(source >= M_MOT_B1){
             motor = source - M_MOT_B1;
             sprintf_s(CMessQ[Qindex].Msg,sizeof(CMessQ[Qindex].Msg), " Mot %d : %s \n", motor,MessPtr[k]);
         }
         else{
              sprintf_s(CMessQ[Qindex].Msg,sizeof(CMessQ[Qindex].Msg), "%s \n",MessPtr[k]);
         }
         //printf("Qindex=%u, k=%d, MessPtr[k]=\"%s\"\n", Qindex, k, MessPtr[k]);
         CMessQ[Qindex].Valid = TRUE;
		 CMessQ[Qindex].Type = Type;
         Qindex++;
         Qindex = Qindex % NUM_MESSAGES;
     }
     return status;
}

/*****************************************************************
 DoBeep() - Outputs a beep tone for 1 second
 ***************************************************************/

int DoBeep(void){
    BOOL bstat;
    int status = OKAY;

    bstat = PlaySound("SystemDefault", NULL, SND_ALIAS | SND_ASYNC | SND_NOSTOP);// TRUE; //Beep(0x500,1000);
    bstat = TRUE;

    if(bstat == FALSE){
        SetErrorBit(M_DCU,DCU_WRN_BEEP_ERROR);
        status = SERR;
    }
    return status;
}


/*******************************************************************
    outputmessages()
        output all error messages at the end of everypass if any exist.
        assume we start at first entry and go to end.
*****************************************************************/

int OutPutMessages(){
     int status = OKAY,k=0;

     while(Qindex > 0){
       if( CMessQ[k].Valid){
             log_printf(CMessQ[k].Type,CMessQ[k].Msg);
             CMessQ[k].Valid = FALSE;
             k++;
             Qindex--;
       }
       else{
            Qindex--;
       }
     }
     //DoBeep();
     return status;
}




/*******************************************************************
    RelayTest()
        Test each relay sequentially to make sure all can open and close
		Parameters:
        reset_test nonzero if we want to start the test over again
*****************************************************************/
int RelayTest(int reset_test)
{
	// Number of cycles while we've passed waiting for a relay to change state
	static int relay_test_cycle_count = 0;
	// Time when we told to relay to switch for calculating timeouts
	static int relay_test_timeout = 0;
	// State of the test state machien
	static int relay_test_state = RELAY_TEST_IDLE;
	// If we're waiting to see an open, close, or nothing
	static int relay_test_wait = RELAY_NO_WAIT;

	// Used for storing the state of the safety circuit
	long int safety_circuit_stat;

	// If we're supposed to restart the test, set everything back to its
	// Initial value
	if (reset_test) {
		relay_test_cycle_count = 0;
		relay_test_timeout = 0;
		relay_test_state = RELAY_TEST_IDLE;
		relay_test_wait = RELAY_NO_WAIT;
		TestBaseRelay = 0;
		TestOuterRelay = 0;
		TestVoltRelay = 0;

		return RELAY_TEST_CONTINUE;
	}


	// When in relay test, the safety circuit should *never* be enabled. If
	// it is, it means the reset relay is stuck closed.
	CheckSafetyCircuit_Errors(&safety_circuit_stat);
	if (safety_circuit_stat) {
		// If we're in here the reset relay is stuck closed
		relay_test_state = RELAY_TEST_IDLE;
		relay_test_wait = 0;
		TestBaseRelay = 0;
		TestOuterRelay = 0;
		TestVoltRelay = 0;
		return RELAY_TEST_FAIL;
	}



	// If we're waiting for a relay to switch, see if it has
	if (relay_test_wait != RELAY_NO_WAIT) {
		relay_test_cycle_count++;

		// If we're waiting and have timed out, show the relay test failed.
		if (GetTickCount() - relay_test_timeout > RELAY_TIMEOUT &&
				relay_test_cycle_count > RELAY_CYCLES) {

			relay_test_state = RELAY_TEST_IDLE;
			relay_test_wait = 0;
			TestBaseRelay = 0;
			TestOuterRelay = 0;
			TestVoltRelay = 0;
			return RELAY_TEST_FAIL;
		}

		// Check if the relay has gone in the desired direction
		if (relay_test_wait == RELAY_OPEN) {
			if (Pusb_state->baseState.SafetyStat & SFT_S_TEST_STAT) {
				return RELAY_TEST_CONTINUE;
			}
		}
		else if (relay_test_wait == RELAY_CLOSE) {
			if (! (Pusb_state->baseState.SafetyStat & SFT_S_TEST_STAT)) {
				return RELAY_TEST_CONTINUE;
			}
		}
		else {  // This indicates an invalid value for relay_test_wait
			SetErrorBit(M_DCU, DCU_ERR_STATE_TRANS);
			log_printf(LOG_ERROR,
					"Invalid relay_test_wait = %d in RelayTest.\n",
					relay_test_wait);
			return RELAY_TEST_FAIL;
		}

	}


	// When we make it here, we are ready to command a new relay to change
	// state.

	relay_test_cycle_count = 0;
	relay_test_timeout = GetTickCount();

	switch (relay_test_state) {
		// idle: start a new test
		case RELAY_TEST_IDLE:
			// Make sure all the relays are closed
			relay_test_wait = RELAY_CLOSE;
			relay_test_state = RELAY_TEST_INIT_DCU;
			return RELAY_TEST_CONTINUE;
			break;
		// Command the DCU relay to open
		case RELAY_TEST_INIT_DCU:
			OpenSafetyCircuit_Errors();
			relay_test_wait = RELAY_OPEN;
			relay_test_state = RELAY_TEST_DCU;
			return RELAY_TEST_CONTINUE;
			break;
		// Command the DCU relay to close
		case RELAY_TEST_DCU:
			CloseSafetyCircuit_Errors();
			relay_test_wait = RELAY_CLOSE;
			relay_test_state = RELAY_TEST_INIT_BASE;
			return RELAY_TEST_CONTINUE;
			break;
		// Command the base relay to open
		case RELAY_TEST_INIT_BASE:
			TestBaseRelay = 1;
			relay_test_wait = RELAY_OPEN;
			relay_test_state = RELAY_TEST_BASE;
			return RELAY_TEST_CONTINUE;
			break;
		// Command the base relay to close
		case RELAY_TEST_BASE:
			TestBaseRelay = 0;
			relay_test_wait = RELAY_CLOSE;
			relay_test_state = RELAY_TEST_INIT_OUTER;
			return RELAY_TEST_CONTINUE;
			break;
		// Command the outer relay to open
		case RELAY_TEST_INIT_OUTER:
			TestOuterRelay = 1;
			relay_test_wait = RELAY_OPEN;
			relay_test_state = RELAY_TEST_OUTER;
			return RELAY_TEST_CONTINUE;
			break;
		// Command the outer relay to close
		case RELAY_TEST_OUTER:
			TestOuterRelay = 0;
			relay_test_wait = RELAY_CLOSE;
			relay_test_state = RELAY_TEST_INIT_VMON;
			return RELAY_TEST_CONTINUE;
			break;
		// Simulate a voltage monitor error
		case RELAY_TEST_INIT_VMON:
			TestVoltRelay = 1;
			relay_test_wait = RELAY_OPEN;
			relay_test_state = RELAY_TEST_VMON;
			return RELAY_TEST_CONTINUE;
			break;
		// Stop simulating the error
		case RELAY_TEST_VMON:
			TestVoltRelay = 0;
			relay_test_wait = RELAY_CLOSE;
			relay_test_state = RELAY_TEST_DONE;
			return RELAY_TEST_CONTINUE;
			break;
		// All done - clean things up and report success
		case RELAY_TEST_DONE:
			relay_test_wait = RELAY_NO_WAIT;
			relay_test_state = RELAY_TEST_IDLE;
			return RELAY_TEST_SUCCESS;
			break;
		// This should never happen - show state transition error
		default:
			SetErrorBit(M_DCU, DCU_ERR_STATE_TRANS);
			log_printf(LOG_ERROR,
					"Invalid relay_test_state = %d in RelayTest.\n",
					relay_test_state);
			return RELAY_TEST_FAIL;
			break;
	}
}

/******************************************************************
  end
  ***********************************************************/


