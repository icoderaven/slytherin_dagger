#include "windows.h"   
#include <stdlib.h>     
#include <stdio.h>  
#include "CardioError.h"
#include "superCommon.h"
#include "ErrorData.h"
#include "keithley.h" 

UINT channel = CHANNEL;
DBL gain = GAIN;
static BOARD board,rboard;

#define CHKERROR(ecode) if ((board.status = (ecode)) != OLNOERROR)\
                  {\
                  olDaReleaseDASS(board.hdass);\
                  olDaTerminate(board.hdrvr);\
                  return SERR;}


BOOL CALLBACK
GetDriver( LPSTR lpszName, LPSTR lpszEntry, LPARAM lParam )   
/*
this is a callback function of olDaEnumBoards, it gets the 
strings of the Open Layers board and attempts to initialize
the board.  If successful, enumeration is halted.
*/

{
   LPBOARD lpboard = (LPBOARD)(LPVOID)lParam;
   
   /* fill in board strings */

   strncpy_s(lpboard->name,MAX_BOARD_NAME_LENGTH-1,lpszName,MAX_BOARD_NAME_LENGTH-1);
   strncpy_s(lpboard->entry,MAX_BOARD_NAME_LENGTH-1,lpszEntry,MAX_BOARD_NAME_LENGTH-1);

   /* try to open board */

   lpboard->status = olDaInitialize(lpszName,&lpboard->hdrvr);
   if   (lpboard->hdrvr != NULL)
      return FALSE;          /* false to stop enumerating */
   else                      
      return TRUE;           /* true to continue          */
}
/********************************************************************
    InitDCUIO - Init the DCU IO board to be used to monitor and
     control the safety circuit
*******************************************************************/

int InitDCUIO(void){

   int status;

   /* Get first available Open Layers board */
   status = OKAY;
   board.hdrvr = NULL;
   CHKERROR(olDaEnumBoards(GetDriver,(LPARAM)(LPBOARD)&board));

   /* check for error within callback function */

   CHKERROR(board.status);

   /* check for NULL driver handle - means no boards */

   if (board.hdrvr == NULL){
      return SERR;
   }
   /* get handle to DIN (digital input) sub system */
   rboard = board;   //just copy the structure, one for input, one for output

   CHKERROR(olDaGetDASS(rboard.hdrvr,OLSS_DIN,0,&rboard.hdass));

   /* set subsystem for single value operation */

   CHKERROR(olDaSetDataFlow(rboard.hdass,OL_DF_SINGLEVALUE));
   CHKERROR(olDaConfig(rboard.hdass));

   /* get handle to DOUT (digital out)sub system */
    CHKERROR(olDaGetDASS(board.hdrvr,OLSS_DOUT,0,&board.hdass));
   /* set subsystem for single value operation */
   CHKERROR(olDaSetDataFlow(board.hdass,OL_DF_SINGLEVALUE));
   CHKERROR(olDaConfig(board.hdass));
   return status;
}

/********************************************************************
    CloseSafetyCircuit() - closes relay so system can operate. Use 
                   digital output 0
********************************************************************/  
 
int CloseSafetyCircuit(){
    long value;
    
   value = ESTOP_RELAY;
   CHKERROR(olDaPutSingleValue(board.hdass,value,channel,gain));
   return OKAY;
}

/*********************************************************************
     OpenSafetyCircuit() - Open the safety circuit causes loss of motor power
                   and brake power.
*********************************************************************/

int OpenSafetyCircuit(){
    long value;
    
   value = 0;
   CHKERROR(olDaPutSingleValue(board.hdass,value,channel,gain));
   return OKAY;
}

/*******************************************************************
       CheckSafetyCircuit(*state) - 1 means closed, 0 means open. 
                                    Use digital in 0
*******************************************************************/      

int CheckSafetyCircuit(long *state){
    long rval=0;
 
   /* get single value */
  
   CHKERROR(olDaGetSingleValue(rboard.hdass,&rval,channel,gain));
   *state = rval & 0x01;
   return OKAY;
}

/*******************************************************************
       CheckEstopButton(*state) - 1 means closed, 0 means open. 
                                    Use digital in 1
*******************************************************************/      

int CheckEstopButton(long *state){
    long rval=0;
 
   /* get single value */
  
   CHKERROR(olDaGetSingleValue(rboard.hdass,&rval,channel,gain));
   *state = rval & 0x02;
   return OKAY;
}

/**********************************************************************
       ShutDownDCUIOBoard()
********************************************************************/

int ShutDownDCUIOBoard(){
 
   /* release the subsystems and the board */
   CHKERROR(olDaReleaseDASS(board.hdass));
   CHKERROR(olDaReleaseDASS(rboard.hdass));
   CHKERROR(olDaTerminate(board.hdrvr));
   /* all done - return */

    return OKAY;
}

/************************************************************
	ResetDCUIOBoard() - Shuts down DCUIO board then initializes it. Returns
						OKAY if both shutdown and init return OKAY.
************************************************************/
int ResetDCUIOBoard() {
	int shutdwn, init;

	shutdwn = ShutDownDCUIOBoard();
	init = InitDCUIO();

	if (shutdwn == SERR || init == SERR) {
		return SERR;
	}
	else {
		return OKAY;
	}
}

