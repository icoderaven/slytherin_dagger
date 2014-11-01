//Keithley hearder file written by jwm

#include <olmem.h>         
#include <olerrors.h>         
#include <oldaapi.h>

/* Error handling macros */

#define STRLEN 80        /* string size for general text manipulation   */
//char str[STRLEN];        /* global string for general text manipulation */

#define ESTOP_RELAY      1   //DO0
#define CHANNEL          0    //channel for keithley device
#define GAIN             1.0


/* simple structure used with board */

typedef struct tag_board {
   HDEV hdrvr;         /* device handle            */
   HDASS hdass;        /* sub system handle        */
   ECODE status;       /* board error status       */
   HBUF  hbuf;         /* sub system buffer handle */
   PWORD lpbuf;        /* buffer pointer           */
   char name[MAX_BOARD_NAME_LENGTH];  /* string for board name    */
   char entry[MAX_BOARD_NAME_LENGTH]; /* string for board name    */
} BOARD;

typedef BOARD* LPBOARD;

int OpenSafetyCircuit();
int InitDCUIO();
int CheckSafetyCircuit(long *state);
int CheckEstopButton(long *state);
int CloseSafetyCircuit();
int ResetDCUIOBoard();
